package frc.lib.SpineUDP;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import frc.lib.SpineUDP.SpineUDP.HalPeriodicThread;
import frc.lib.SpineUDP.SpineUDP.SpineClock;
import frc.lib.SpineUDP.SpineUDP.OdometrySender;
import frc.lib.SpineUDP.SpineUDP.TriggerRing;
import frc.lib.SpineUDP.SpineUDP.TriggerTableSender;
import frc.lib.SpineUDP.SpineUDP.VisionLinkProtocol;
import frc.lib.SpineUDP.SpineUDP.VisionPoseReceiver;
import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.SocketTimeoutException;
import java.nio.ByteBuffer;
import java.nio.channels.DatagramChannel;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HexFormat;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.ObjIntConsumer;
import java.util.stream.Stream;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class SpineUDPTest {
  private static final String kLoopback = "127.0.0.1";
  private static final String kStatsTable = "VisionLinkTest";

  @BeforeAll
  static void initHal() {
    assertTrue(HAL.initialize(500, 0));
  }

  // -------------------------------------------------------------------------------------------
  // Stream 1
  // -------------------------------------------------------------------------------------------

  @Test
  void odometrySenderSendsSampledPacketsAtRate() throws Exception {
    try (DatagramChannel rx = DatagramChannel.open().bind(new InetSocketAddress(kLoopback, 0))) {
      int port = ((InetSocketAddress) rx.getLocalAddress()).getPort();

      try (OdometrySender sender = new OdometrySender(kLoopback, port, 200, kStatsTable, 1.0)) {
        sender.start(
            sample -> {
              sample.set(SpineClock.now(), 1.5, -2.25, 0.75, 0.1, 0.2, 0.3);
              return true;
            });

        ByteBuffer buf = ByteBuffer.allocate(128).order(VisionLinkProtocol.ORDER);
        long firstSeq = -1;
        for (int i = 0; i < 50; i++) {
          buf.clear();
          receiveBlocking(rx, buf);
          buf.flip();
          assertEquals(VisionLinkProtocol.ODOM_SIZE, buf.remaining());
          long seq = buf.getLong(VisionLinkProtocol.ODOM_OFF_SEQ);
          if (firstSeq < 0) {
            firstSeq = seq;
          }
          assertEquals(firstSeq + i, seq);
          assertEquals(1.5, buf.getDouble(VisionLinkProtocol.ODOM_OFF_X));
          assertEquals(-2.25, buf.getDouble(VisionLinkProtocol.ODOM_OFF_Y));
          assertEquals(0.75, buf.getDouble(VisionLinkProtocol.ODOM_OFF_THETA));
          assertEquals(0.1, buf.getDouble(VisionLinkProtocol.ODOM_OFF_VX));
          assertEquals(0.2, buf.getDouble(VisionLinkProtocol.ODOM_OFF_VY));
          assertEquals(0.3, buf.getDouble(VisionLinkProtocol.ODOM_OFF_OMEGA));
          double age = SpineClock.now() - buf.getDouble(VisionLinkProtocol.ODOM_OFF_TIMESTAMP);
          assertTrue(age >= 0 && age < 0.5, "timestamp should come from SpineClock, age=" + age);
        }

        // Rate: count packets over one second of controller time.
        double start = SpineClock.now();
        long startSent = sender.getSentCount();
        Thread.sleep(1000);
        double elapsed = SpineClock.now() - start;
        double rate = (sender.getSentCount() - startSent) / elapsed;
        assertTrue(rate > 180 && rate < 220, "expected ~200 Hz, got " + rate);
      }
    }
  }

  @Test
  void halPeriodicThreadHoldsSchedule() throws Exception {
    long[] ticks = new long[1];
    HalPeriodicThread thread = new HalPeriodicThread("test-periodic", 0.005, () -> ticks[0]++, 0);
    thread.start();
    Thread.sleep(2000);
    thread.stop();
    double rate = ticks[0] / 2.0;
    assertTrue(rate > 185 && rate < 215, "expected ~200 Hz, got " + rate);
    assertTrue(
        thread.getLateTickCount() < 10, "late ticks on an idle desktop: " + thread.getLateTickCount());
  }

  // -------------------------------------------------------------------------------------------
  // Stream 2
  // -------------------------------------------------------------------------------------------

  @Test
  void visionPoseReceiverPassesTimestampVerbatimAndCountsDrops() throws Exception {
    try (VisionPoseReceiver receiver = new VisionPoseReceiver(0, 16, 1 << 16, 1.5, kStatsTable, 1.0);
        DatagramChannel tx = DatagramChannel.open()) {
      receiver.start();
      InetSocketAddress target = new InetSocketAddress(kLoopback, receiver.getBoundPort());
      ByteBuffer pkt = VisionLinkProtocol.allocate(VisionLinkProtocol.POSE_SIZE);

      double goodTs = SpineClock.now() - 0.05;
      sendPose(tx, target, pkt, 7, goodTs, 1.0, 2.0, 0.5);
      sendPose(tx, target, pkt, 8, SpineClock.now() + 1.0, 1.0, 2.0, 0.5); // future
      sendPose(tx, target, pkt, 9, SpineClock.now() - 5.0, 1.0, 2.0, 0.5); // too old
      sendPose(tx, target, pkt, 10, Double.NaN, 1.0, 2.0, 0.5); // NaN timestamp
      sendPose(tx, target, pkt, 11, SpineClock.now(), Double.NaN, 2.0, 0.5); // NaN pose
      tx.send(ByteBuffer.allocate(VisionLinkProtocol.POSE_SIZE + 8), target); // wrong size

      waitFor(() -> receiver.getRecvCount() + receiver.getDroppedStale() + receiver.getDroppedSizeMismatch() >= 6);
      assertEquals(1, receiver.getRecvCount());
      assertEquals(4, receiver.getDroppedStale());
      assertEquals(1, receiver.getDroppedSizeMismatch());

      assertEquals(1, receiver.readPackets());
      AtomicReference<double[]> seen = new AtomicReference<>();
      receiver.forEachPacket(
          (pose, i) ->
              seen.set(
                  new double[] {pose.getSeq(), pose.getTimestamp(), pose.getX(), pose.getY(), pose.getTheta()}));
      double[] v = seen.get();
      assertEquals(7, (long) v[0]);
      assertEquals(Double.doubleToRawLongBits(goodTs), Double.doubleToRawLongBits(v[1]));
      assertEquals(1.0, v[2]);
      assertEquals(2.0, v[3]);
      assertEquals(0.5, v[4]);
    }
  }

  // -------------------------------------------------------------------------------------------
  // Stream 3
  // -------------------------------------------------------------------------------------------

  @Test
  void triggerRingOrdersMostRecentFirstWithSentinels() {
    TriggerRing ring = new TriggerRing();
    ByteBuffer buf = VisionLinkProtocol.allocate(VisionLinkProtocol.TRIG_SIZE);

    ring.record(10.0);
    ring.record(10.5);
    ring.writeInto(buf, 0, 42);
    assertEquals(42, buf.getLong(VisionLinkProtocol.TRIG_OFF_PACKET_SEQ));
    assertEquals(1, buf.getLong(VisionLinkProtocol.trigOffTriggerSeq(0)));
    assertEquals(10.5, buf.getDouble(VisionLinkProtocol.trigOffTimestamp(0)));
    assertEquals(0, buf.getLong(VisionLinkProtocol.trigOffTriggerSeq(1)));
    assertEquals(10.0, buf.getDouble(VisionLinkProtocol.trigOffTimestamp(1)));
    for (int i = 2; i < VisionLinkProtocol.TRIG_ENTRY_COUNT; i++) {
      assertEquals(TriggerRing.EMPTY_TRIGGER_SEQ, buf.getLong(VisionLinkProtocol.trigOffTriggerSeq(i)));
      assertEquals(0.0, buf.getDouble(VisionLinkProtocol.trigOffTimestamp(i)));
    }

    for (int i = 2; i < 12; i++) {
      ring.record(10.0 + 0.5 * i);
    }
    ring.writeInto(buf, 0, 43);
    for (int i = 0; i < VisionLinkProtocol.TRIG_ENTRY_COUNT; i++) {
      long seq = buf.getLong(VisionLinkProtocol.trigOffTriggerSeq(i));
      assertEquals(11 - i, seq);
      assertEquals(10.0 + 0.5 * seq, buf.getDouble(VisionLinkProtocol.trigOffTimestamp(i)));
    }
    assertEquals(12, ring.getNextTriggerSeq());
  }

  /**
   * One writer hammers the ring as fast as it can while a reader copies it continuously. Every
   * snapshot the reader accepts must be internally consistent: consecutive descending sequence
   * numbers, each with exactly the timestamp the writer paired with it.
   */
  @Test
  void seqlockNeverYieldsTornSnapshot() throws Exception {
    TriggerRing ring = new TriggerRing();
    AtomicBoolean stop = new AtomicBoolean();
    Thread writer =
        new Thread(
            () -> {
              long n = 0;
              while (!stop.get()) {
                ring.record(n * 0.25); // timestamp is a pure function of the sequence number
                n++;
              }
            });
    writer.start();

    ByteBuffer buf = VisionLinkProtocol.allocate(VisionLinkProtocol.TRIG_SIZE);
    long snapshots = 0;
    long retries = 0;
    long deadline = System.currentTimeMillis() + 1000;
    try {
      while (System.currentTimeMillis() < deadline) {
        retries += ring.writeInto(buf, 0, snapshots);
        long newest = buf.getLong(VisionLinkProtocol.trigOffTriggerSeq(0));
        for (int i = 0; i < VisionLinkProtocol.TRIG_ENTRY_COUNT; i++) {
          long seq = buf.getLong(VisionLinkProtocol.trigOffTriggerSeq(i));
          double ts = buf.getDouble(VisionLinkProtocol.trigOffTimestamp(i));
          if (seq == TriggerRing.EMPTY_TRIGGER_SEQ) {
            assertEquals(0.0, ts);
            continue;
          }
          assertEquals(newest - i, seq, "entries must be consecutive, snapshot " + snapshots);
          assertEquals(seq * 0.25, ts, "timestamp must match its sequence number");
        }
        snapshots++;
      }
    } finally {
      stop.set(true);
      writer.join();
    }
    System.out.println("[seqlock] " + snapshots + " snapshots, " + retries + " retries");
    assertTrue(snapshots > 1000);
  }

  @Test
  void triggerTableSenderDeliversRecordedTriggers() throws Exception {
    try (DatagramChannel rx = DatagramChannel.open().bind(new InetSocketAddress(kLoopback, 0))) {
      int port = ((InetSocketAddress) rx.getLocalAddress()).getPort();
      try (TriggerTableSender sender = new TriggerTableSender(kLoopback, port, kStatsTable, 1.0)) {
        sender.start();
        double ts = SpineClock.now();
        assertEquals(0, sender.recordTrigger(ts));

        ByteBuffer buf = ByteBuffer.allocate(128).order(VisionLinkProtocol.ORDER);
        receiveBlocking(rx, buf);
        buf.flip();
        assertEquals(VisionLinkProtocol.TRIG_SIZE, buf.remaining());
        assertEquals(0, buf.getLong(VisionLinkProtocol.trigOffTriggerSeq(0)));
        assertEquals(
            Double.doubleToRawLongBits(ts),
            Double.doubleToRawLongBits(buf.getDouble(VisionLinkProtocol.trigOffTimestamp(0))));
      }
    }
  }

  // -------------------------------------------------------------------------------------------
  // Zero allocation
  // -------------------------------------------------------------------------------------------

  /**
   * Runs all three streams hot, then checks that none of the link's threads allocates in steady
   * state: every thread must have at least one completely clean measurement window, and each window
   * includes stats publishes.
   *
   * <p>This is a desktop proxy that catches code regressions (a lambda, boxing or varargs on a hot
   * path allocates on every packet, so it dirties every window). The ground truth on the robot is
   * the {@code *ThreadAllocBytes} gauges under {@code VisionLink/*}, which should stop climbing.
   */
  @Test
  void linkThreadsDoNotAllocateInSteadyState() throws Exception {
    com.sun.management.ThreadMXBean mx =
        (com.sun.management.ThreadMXBean) java.lang.management.ManagementFactory.getThreadMXBean();
    mx.setThreadAllocatedMemoryEnabled(true);

    try (DatagramChannel sink = DatagramChannel.open().bind(new InetSocketAddress(kLoopback, 0));
        DatagramChannel poseTx = DatagramChannel.open();
        VisionPoseReceiver receiver = new VisionPoseReceiver(0, 64, 1 << 16, 1.5, kStatsTable, 0.25);
        OdometrySender odom =
            new OdometrySender(
                kLoopback, ((InetSocketAddress) sink.getLocalAddress()).getPort(), 500, kStatsTable, 0.25);
        TriggerTableSender trig =
            new TriggerTableSender(
                kLoopback, ((InetSocketAddress) sink.getLocalAddress()).getPort(), kStatsTable, 0.25)) {
      receiver.start();
      trig.start();
      odom.start(
          sample -> {
            sample.set(SpineClock.now(), 1, 2, 3, 4, 5, 6);
            return true;
          });

      // The trigger producer must be a single thread; this periodic thread plays the pulser.
      HalPeriodicThread pulser = new HalPeriodicThread("test-pulser", 1.0 / 120, trig::recordTrigger, 0);
      pulser.start();

      // Drain the sink so the OS never pushes back, and drain the receiver like the robot loop does.
      ObjIntConsumer<VisionPoseReceiver.PoseView> consume = (pose, i) -> {};
      InetSocketAddress poseTarget = new InetSocketAddress(kLoopback, receiver.getBoundPort());
      ByteBuffer pkt = VisionLinkProtocol.allocate(VisionLinkProtocol.POSE_SIZE);
      ByteBuffer drain = ByteBuffer.allocateDirect(256);
      sink.configureBlocking(false);

      Runnable pump =
          () -> {
            try {
              sendPose(poseTx, poseTarget, pkt, 1, SpineClock.now() - 0.01, 1, 2, 3);
              drain.clear();
              while (sink.receive(drain) != null) {
                drain.clear();
              }
              if (receiver.readPackets() > 0) {
                receiver.forEachPacket(consume);
              }
            } catch (IOException e) {
              throw new RuntimeException(e);
            }
          };

      // Warm up until C2 has compiled the hot paths. Interpreted/C1 JDK socket code allocates a few
      // small temporaries that C2's escape analysis removes, so measuring earlier shows a one-off
      // ~200 B per path that never recurs. Also covers the first NT publish, which creates entries.
      pumpFor(pump, 6000);

      long[] tids = {
        odom.getPeriodicThread().getThread().getId(),
        trig.getThread().getId(),
        receiver.getThread().getId(),
        pulser.getThread().getId()
      };
      String[] names = {"odometry", "trigger-send", "pose-recv", "pulser"};
      // Five consecutive windows. A real per-packet allocation shows up in every window, because
      // each one carries hundreds of packets per thread. A path that reaches C2 late shows up as a
      // one-off in a single window. When other tests have run first in the same JVM, that one-off
      // can land in any window (seen: 184-344 B once, ~10 s in, then clean), so the requirement is
      // at least one fully clean window per thread, not a clean last window.
      final int windows = 5;
      StringBuilder report = new StringBuilder();
      boolean[] hadCleanWindow = new boolean[tids.length];
      for (int w = 0; w < windows; w++) {
        long[] before = mx.getThreadAllocatedBytes(tids);
        pumpFor(pump, 2000);
        long[] after = mx.getThreadAllocatedBytes(tids);
        report.append("window ").append(w).append(": ");
        for (int i = 0; i < tids.length; i++) {
          long delta = after[i] - before[i];
          hadCleanWindow[i] |= delta == 0;
          report.append(names[i]).append('=').append(delta).append("B ");
        }
        report.append('\n');
      }
      pulser.stop();
      System.out.println("[alloc]\n" + report);

      boolean clean = true;
      for (boolean ok : hadCleanWindow) {
        clean &= ok;
      }
      assertTrue(odom.getSentCount() > 1000, "odometry should have been running");
      assertTrue(receiver.getRecvCount() > 100, "receiver should have been running");
      assertTrue(trig.getSentCount() > 100, "trigger sender should have been running");
      assertTrue(clean, "a link thread allocated in every window (steady-state allocation): " + report);
    }
  }

  // -------------------------------------------------------------------------------------------
  // Golden vectors shared with the C++ end
  // -------------------------------------------------------------------------------------------

  /**
   * Encodes every packet in {@code ros_ws/src/spine_link/test/data/spine-link-vectors.txt} and compares it byte-for-byte with
   * the expected wire bytes, then decodes the expected bytes field by field, bit-for-bit. The C++
   * tests check the same file, so the two ends can't drift apart silently.
   */
  @Test
  void goldenVectorsMatchSharedFile() throws IOException {
    List<String> lines =
        Files.readAllLines(Path.of("ros_ws", "src", "spine_link", "test", "data", "spine-link-vectors.txt"), StandardCharsets.US_ASCII);
    Map<String, Integer> counts = new LinkedHashMap<>();
    for (String raw : lines) {
      String line = raw.strip();
      if (line.isEmpty() || line.startsWith("#")) {
        continue;
      }
      String[] tok = line.split("\\s+");
      String kind = tok[0];
      String what = kind + " " + tok[1];
      Map<String, String> f = new LinkedHashMap<>();
      for (int i = 2; i < tok.length; i++) {
        int eq = tok[i].indexOf('=');
        f.put(tok[i].substring(0, eq), tok[i].substring(eq + 1));
      }
      byte[] expected = HexFormat.of().parseHex(f.remove("expect"));

      ByteBuffer buf;
      switch (kind) {
        case "odom" -> {
          buf = VisionLinkProtocol.allocate(VisionLinkProtocol.ODOM_SIZE);
          VisionLinkProtocol.encodeOdometry(
              buf, 0, lng(f, "seq"), dbl(f, "timestamp"), dbl(f, "x"), dbl(f, "y"),
              dbl(f, "theta"), dbl(f, "vx"), dbl(f, "vy"), dbl(f, "omega"));
        }
        case "pose" -> {
          buf = VisionLinkProtocol.allocate(VisionLinkProtocol.POSE_SIZE);
          VisionLinkProtocol.encodeVisionPose(
              buf, 0, lng(f, "seq"), dbl(f, "timestamp"), dbl(f, "x"), dbl(f, "y"),
              dbl(f, "theta"));
        }
        case "trig" -> {
          // Rebuild the table through the real TriggerRing: record triggers 0..newest, giving
          // those still in the packet their vector timestamps.
          buf = VisionLinkProtocol.allocate(VisionLinkProtocol.TRIG_SIZE);
          TriggerRing ring = new TriggerRing();
          long newest = lng(f, "triggerSeq[0]");
          for (long s = 0; s <= newest; s++) {
            int i = (int) (newest - s);
            ring.record(i < VisionLinkProtocol.TRIG_ENTRY_COUNT ? dbl(f, "timestamp[" + i + "]") : 0.0);
          }
          ring.writeInto(buf, 0, lng(f, "packetSeq"));
        }
        default -> throw new AssertionError("unknown stream '" + kind + "' in " + what);
      }
      byte[] actual = new byte[buf.capacity()];
      buf.get(0, actual);
      assertEquals(expected.length, actual.length, what + " size");
      assertArrayEquals(expected, actual, what + " encode");

      ByteBuffer wire = ByteBuffer.wrap(expected).order(VisionLinkProtocol.ORDER);
      int covered = 0;
      for (Map.Entry<String, String> e : f.entrySet()) {
        int off = wireOffset(kind, e.getKey());
        if (isIntField(e.getKey())) {
          assertEquals(Long.parseLong(e.getValue()), wire.getLong(off), what + " " + e.getKey());
        } else {
          assertEquals(
              Double.doubleToRawLongBits(Double.parseDouble(e.getValue())),
              Double.doubleToRawLongBits(wire.getDouble(off)),
              what + " " + e.getKey());
        }
        covered += 8;
      }
      assertEquals(expected.length, covered, what + " must list every field");
      counts.merge(kind, 1, Integer::sum);
    }
    System.out.println("[vectors] " + counts);
    for (String kind : new String[] {"odom", "pose", "trig"}) {
      assertTrue(counts.getOrDefault(kind, 0) > 0, "no " + kind + " vectors");
    }
  }

  private static boolean isIntField(String name) {
    return name.equals("seq") || name.equals("packetSeq") || name.startsWith("triggerSeq[");
  }

  private static int wireOffset(String kind, String name) {
    if (kind.equals("trig")) {
      if (name.equals("packetSeq")) {
        return VisionLinkProtocol.TRIG_OFF_PACKET_SEQ;
      }
      int i = Integer.parseInt(name.substring(name.indexOf('[') + 1, name.indexOf(']')));
      return name.startsWith("triggerSeq[")
          ? VisionLinkProtocol.trigOffTriggerSeq(i)
          : VisionLinkProtocol.trigOffTimestamp(i);
    }
    boolean odom = kind.equals("odom");
    return switch (name) {
      case "seq" -> odom ? VisionLinkProtocol.ODOM_OFF_SEQ : VisionLinkProtocol.POSE_OFF_SEQ;
      case "timestamp" -> odom ? VisionLinkProtocol.ODOM_OFF_TIMESTAMP : VisionLinkProtocol.POSE_OFF_TIMESTAMP;
      case "x" -> odom ? VisionLinkProtocol.ODOM_OFF_X : VisionLinkProtocol.POSE_OFF_X;
      case "y" -> odom ? VisionLinkProtocol.ODOM_OFF_Y : VisionLinkProtocol.POSE_OFF_Y;
      case "theta" -> odom ? VisionLinkProtocol.ODOM_OFF_THETA : VisionLinkProtocol.POSE_OFF_THETA;
      case "vx" -> VisionLinkProtocol.ODOM_OFF_VX;
      case "vy" -> VisionLinkProtocol.ODOM_OFF_VY;
      case "omega" -> VisionLinkProtocol.ODOM_OFF_OMEGA;
      default -> throw new AssertionError("unknown field '" + name + "' for " + kind);
    };
  }

  private static long lng(Map<String, String> f, String key) {
    String v = f.get(key);
    if (v == null) {
      throw new AssertionError("missing field " + key);
    }
    return Long.parseLong(v);
  }

  private static double dbl(Map<String, String> f, String key) {
    String v = f.get(key);
    if (v == null) {
      throw new AssertionError("missing field " + key);
    }
    return Double.parseDouble(v); // accepts decimal and C99 hex-float
  }

  // -------------------------------------------------------------------------------------------
  // Clock rule and portability
  // -------------------------------------------------------------------------------------------

  /**
   * Enforces the one-clock rule and the no-robot-imports rule on the library source itself, so a
   * later edit can't quietly reintroduce a second time source.
   */
  @Test
  void libraryUsesOnlyTheSharedClockAndNoRobotCode() throws IOException {
    Path dir = Path.of("src", "main", "java", "frc", "lib", "SpineUDP");
    String[] forbidden = {
      "System.nanoTime", "System.currentTimeMillis", "getFPGATime", "getFPGATimestamp", "Timer.", "frc.robot"
    };
    try (Stream<Path> files = Files.list(dir)) {
      for (Path file : (Iterable<Path>) files::iterator) {
        // Code only: the javadoc names the forbidden calls on purpose.
        String src =
            Files.readString(file, StandardCharsets.UTF_8)
                .replaceAll("(?s)/\\*.*?\\*/", "")
                .replaceAll("//[^\\n]*", "");
        for (String token : forbidden) {
          assertTrue(!src.contains(token), file.getFileName() + " contains forbidden '" + token + "'");
        }
      }
    }
  }

  // -------------------------------------------------------------------------------------------
  // Helpers
  // -------------------------------------------------------------------------------------------

  private static void sendPose(
      DatagramChannel tx, InetSocketAddress target, ByteBuffer pkt, long seq, double ts, double x, double y, double theta)
      throws IOException {
    VisionLinkProtocol.encodeVisionPose(pkt, 0, seq, ts, x, y, theta);
    pkt.clear();
    tx.send(pkt, target);
  }

  private static void receiveBlocking(DatagramChannel rx, ByteBuffer buf) throws IOException {
    // Bounded non-blocking poll, so a missing packet fails the test instead of hanging it.
    rx.configureBlocking(false);
    long deadline = System.currentTimeMillis() + 2000;
    while (rx.receive(buf) == null) {
      if (System.currentTimeMillis() > deadline) {
        throw new SocketTimeoutException("no datagram within 2 s");
      }
      Thread.onSpinWait();
    }
  }

  private static void waitFor(java.util.function.BooleanSupplier condition) throws InterruptedException {
    long deadline = System.currentTimeMillis() + 2000;
    while (!condition.getAsBoolean() && System.currentTimeMillis() < deadline) {
      Thread.sleep(5);
    }
  }

  private static void pumpFor(Runnable pump, long millis) throws InterruptedException {
    long end = System.currentTimeMillis() + millis;
    while (System.currentTimeMillis() < end) {
      pump.run();
      Thread.sleep(2);
    }
  }
}
