// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.SpineUDP;

import edu.wpi.first.hal.NotifierJNI;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Threads;

import java.io.IOException;
import java.lang.invoke.MethodHandles;
import java.lang.invoke.VarHandle;
import java.net.InetSocketAddress;
import java.net.StandardSocketOptions;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.channels.ClosedChannelException;
import java.nio.channels.DatagramChannel;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.LockSupport;
import java.util.function.ObjIntConsumer;

/**
 * The robot-controller end of the controller&harr;Orin vision link, gathered into one file.
 *
 * <p>Three independent UDP streams, each with its own socket and port:
 *
 * <ul>
 *   <li>{@link OdometrySender} &mdash; Stream 1, swerve odometry, controller &rarr; Orin, 64 B.
 *   <li>{@link VisionPoseReceiver} &mdash; Stream 2, fused vision poses, Orin &rarr; controller, 40 B.
 *   <li>{@link TriggerTableSender} &mdash; Stream 3, camera trigger table, controller &rarr; Orin, 88
 *       B.
 * </ul>
 *
 * <h2>Rules this library holds itself to</h2>
 *
 * <ul>
 *   <li><b>One clock.</b> Every time value &mdash; wire timestamps, staleness checks, notifier
 *       deadlines, stats cadence &mdash; comes from {@link SpineClock#now()}, which is {@link
 *       MathSharedStore#getTimestamp()}. Nothing here reads {@code System.nanoTime}, {@code
 *       System.currentTimeMillis}, {@code Timer} or the FPGA directly. The Orin never originates a
 *       timestamp; it only echoes back ones this controller produced.
 *   <li><b>Zero allocation in steady state.</b> Buffers, samples and counters are allocated once.
 *       Each stats publisher reports its own thread's allocated bytes so this can be checked on the
 *       robot.
 *   <li><b>Lock-free.</b> Periodic threads drive the HAL notifier directly (no {@code Notifier}
 *       wrapper lock); Stream 3 hands off through a seqlock; Stream 2 through an SPSC ring; counters
 *       are single-writer opaque longs.
 *   <li><b>Portable.</b> No {@code frc.robot} imports; callers pass a {@link Config}. The only WPILib
 *       touchpoints are {@link SpineClock}, {@link HalPeriodicThread} and {@link StatsPublisher}, so
 *       moving to SystemCore touches only those.
 * </ul>
 *
 * <p>This is a namespace only; it is never instantiated.
 */
public final class SpineUDP {

  // ===========================================================================================
  // Config
  // ===========================================================================================

  /**
   * Immutable link configuration, supplied by the robot project so this library never depends on
   * any particular robot's constants.
   *
   * @param orinHost Orin address for Streams 1 and 3
   * @param odometryPort Stream 1 destination port on the Orin
   * @param visionPosePort Stream 2 port bound on this controller
   * @param triggerTablePort Stream 3 destination port on the Orin
   * @param odometryRateHz Stream 1 send rate
   * @param visionRingSize Stream 2 ring capacity in packets; a power of two
   * @param visionSocketRcvBufBytes Stream 2 socket receive buffer size
   * @param visionMaxAgeSeconds Stream 2 poses older than this are dropped and counted
   * @param statsPublishPeriodSeconds how often each stream republishes its counters
   * @param statsTable NetworkTables table the counters are published under
   */
  public record Config(
      String orinHost,
      int odometryPort,
      int visionPosePort,
      int triggerTablePort,
      int odometryRateHz,
      int visionRingSize,
      int visionSocketRcvBufBytes,
      double visionMaxAgeSeconds,
      double statsPublishPeriodSeconds,
      String statsTable) {
    /** Validates the configuration. */
    public Config {
      if (odometryRateHz <= 0) {
        throw new IllegalArgumentException("odometryRateHz must be positive, got " + odometryRateHz);
      }
      if (visionRingSize < 2 || Integer.bitCount(visionRingSize) != 1) {
        throw new IllegalArgumentException(
            "visionRingSize must be a power of two, got " + visionRingSize);
      }
      if (!(visionMaxAgeSeconds > 0) || !(statsPublishPeriodSeconds > 0)) {
        throw new IllegalArgumentException("periods and ages must be positive");
      }
    }
  }

  // ===========================================================================================
  // SpineClock
  // ===========================================================================================

  /**
   * The single time source for the whole link: {@link MathSharedStore#getTimestamp()}.
   *
   * <p>On the roboRIO and on SystemCore this is the controller's own clock &mdash; the same timebase
   * WPILib's pose estimators and the HAL notifier use. Every timestamp that goes on the wire, every
   * staleness check and every scheduling deadline in this library goes through here, so there is
   * exactly one clock in the system and nothing to synchronize.
   */
  public static final class SpineClock {
    /**
     * Returns the controller timestamp.
     *
     * @return the current time, in seconds
     */
    public static double now() {
      return MathSharedStore.getTimestamp();
    }

    /**
     * Returns the controller timestamp in whole microseconds, the unit the HAL notifier takes.
     *
     * @return the current time, in microseconds
     */
    public static long nowMicros() {
      return (long) (MathSharedStore.getTimestamp() * 1e6);
    }

    private SpineClock() {}
  }

  // ===========================================================================================
  // VisionLinkProtocol
  // ===========================================================================================

  /**
   * Single source of truth for the controller&harr;Orin vision-link wire format.
   *
   * <p>Three independent UDP streams, each with its own socket, port and thread. Every field is an
   * 8-byte {@code long} or {@code double}, so every offset is a multiple of 8 and there are no
   * alignment or padding concerns anywhere in this protocol.
   *
   * <p><b>This class deliberately has zero WPILib imports.</b> It is pure {@code java.nio}, so the
   * wire format stays unit-testable on a desktop JVM without the HAL.
   *
   * <p><b>Byte order is explicitly {@link ByteOrder#LITTLE_ENDIAN} on both ends, never "native
   * order."</b> Fields are read and written with absolute indexed get/put at the hardcoded offsets
   * below &mdash; never a relative read, never a struct cast. That last point matters most on the
   * Orin side, where a C/C++ struct cast would let compiler padding silently corrupt the layout.
   *
   * <p>Note that this is <b>not</b> the format used by {@code UdpLatencySubsystem}, the separate
   * RoboRIO&harr;Orange&nbsp;Pi bench latency rig on port 5810. That rig is big-endian and unrelated;
   * the two formats must not be harmonized.
   *
   * <h2>Stream 1 &mdash; Swerve Odometry (controller &rarr; Orin, 200 Hz, 64 bytes)</h2>
   *
   * <pre>
   *   0  seq            long
   *   8  timestamp      double
   *  16  x              double
   *  24  y              double
   *  32  theta          double
   *  40  vx             double
   *  48  vy             double
   *  56  omega          double
   * </pre>
   *
   * <p>The pose is pure wheel+gyro dead reckoning in an arbitrary, never-reset frame. It is never the
   * vision-corrected pose: feeding the Orin's own output back to it as "odometry" would close a loop.
   *
   * <h2>Stream 2 &mdash; Vision Pose (Orin &rarr; controller, event-driven, 40 bytes)</h2>
   *
   * <pre>
   *   0  seq            long
   *   8  timestamp      double
   *  16  x              double
   *  24  y              double
   *  32  theta          double
   * </pre>
   *
   * <p>{@code timestamp} is the controller-originated timestamp of the trigger that produced the
   * pose, sourced from the Stream 3 table and echoed back <b>verbatim</b>. It is never recomputed or
   * approximated on the Orin side, and never adjusted on the way back in. This is the single most
   * important correctness requirement in the protocol: pose fusion needs the true controller-timebase
   * timestamp for every frame, and a wrong value degrades accuracy silently.
   *
   * <h2>Stream 3 &mdash; Trigger Table (controller &rarr; Orin, per camera trigger pulse, 88
   * bytes)</h2>
   *
   * <pre>
   *   0            packetSeq      long
   *   8 + i*16     triggerSeq     long     (i = 0..4, most recent trigger first)
   *  16 + i*16     timestamp      double
   * </pre>
   *
   * <p>Each packet carries the 5 most recent trigger&rarr;timestamp pairs rather than just the
   * newest, so a single dropped datagram never loses a timestamp &mdash; the value simply arrives
   * again in the next packet. TCP was considered and rejected: its retransmission plus strict
   * in-order delivery head-of-line-blocks every later packet until a lost one is resent, which for a
   * periodic time-critical signal is worse than occasionally losing one sample.
   */
  public static final class VisionLinkProtocol {
    /** Wire byte order for all three streams. Hardcoded on both ends; never "native order." */
    public static final ByteOrder ORDER = ByteOrder.LITTLE_ENDIAN;

    // -----------------------------------------------------------------------------------------
    // Stream 1 - Swerve Odometry (controller -> Orin)
    // -----------------------------------------------------------------------------------------

    /** Total size of a Stream 1 odometry packet, in bytes. */
    public static final int ODOM_SIZE = 64;

    public static final int ODOM_OFF_SEQ = 0;
    public static final int ODOM_OFF_TIMESTAMP = 8;
    public static final int ODOM_OFF_X = 16;
    public static final int ODOM_OFF_Y = 24;
    public static final int ODOM_OFF_THETA = 32;
    public static final int ODOM_OFF_VX = 40;
    public static final int ODOM_OFF_VY = 48;
    public static final int ODOM_OFF_OMEGA = 56;

    // -----------------------------------------------------------------------------------------
    // Stream 2 - Vision Pose (Orin -> controller)
    // -----------------------------------------------------------------------------------------

    /** Total size of a Stream 2 vision-pose packet, in bytes. */
    public static final int POSE_SIZE = 40;

    public static final int POSE_OFF_SEQ = 0;
    public static final int POSE_OFF_TIMESTAMP = 8;
    public static final int POSE_OFF_X = 16;
    public static final int POSE_OFF_Y = 24;
    public static final int POSE_OFF_THETA = 32;

    // -----------------------------------------------------------------------------------------
    // Stream 3 - Trigger Table (controller -> Orin)
    // -----------------------------------------------------------------------------------------

    /** Number of trigger entries carried in every Stream 3 packet (deliberate redundancy). */
    public static final int TRIG_ENTRY_COUNT = 5;

    /** Byte offset of the first trigger entry, i.e. the size of the Stream 3 header. */
    public static final int TRIG_ENTRY_BASE = 8;

    /** Byte distance between consecutive trigger entries. */
    public static final int TRIG_ENTRY_STRIDE = 16;

    /** Total size of a Stream 3 trigger-table packet, in bytes: 8 + 5 * 16. */
    public static final int TRIG_SIZE = TRIG_ENTRY_BASE + TRIG_ENTRY_COUNT * TRIG_ENTRY_STRIDE;

    public static final int TRIG_OFF_PACKET_SEQ = 0;

    /**
     * Returns the byte offset of entry {@code i}'s {@code triggerSeq} field: {@code 8 + i * 16}.
     *
     * @param i entry index, 0 (most recent) through 4
     * @return the byte offset within a Stream 3 packet
     */
    public static int trigOffTriggerSeq(int i) {
      return TRIG_ENTRY_BASE + i * TRIG_ENTRY_STRIDE;
    }

    /**
     * Returns the byte offset of entry {@code i}'s {@code timestamp} field: {@code 16 + i * 16}.
     *
     * @param i entry index, 0 (most recent) through 4
     * @return the byte offset within a Stream 3 packet
     */
    public static int trigOffTimestamp(int i) {
      return TRIG_ENTRY_BASE + i * TRIG_ENTRY_STRIDE + 8;
    }

    // -----------------------------------------------------------------------------------------
    // Encoders - absolute indexed access only, so the caller's buffer position and limit are never
    // disturbed and one buffer can back many packet slots.
    // -----------------------------------------------------------------------------------------

    /**
     * Writes a Stream 1 odometry packet into {@code dst} starting at {@code base}.
     *
     * @param dst destination buffer, which must already be in {@link #ORDER}
     * @param base byte offset of the start of the packet within {@code dst}
     * @param seq monotonically increasing packet sequence number
     * @param timestamp controller timestamp, in seconds
     * @param x X position, in meters
     * @param y Y position, in meters
     * @param theta heading, in radians
     * @param vx field-frame X velocity, in meters per second
     * @param vy field-frame Y velocity, in meters per second
     * @param omega angular velocity, in radians per second
     */
    public static void encodeOdometry(
        ByteBuffer dst,
        int base,
        long seq,
        double timestamp,
        double x,
        double y,
        double theta,
        double vx,
        double vy,
        double omega) {
      dst.putLong(base + ODOM_OFF_SEQ, seq);
      dst.putDouble(base + ODOM_OFF_TIMESTAMP, timestamp);
      dst.putDouble(base + ODOM_OFF_X, x);
      dst.putDouble(base + ODOM_OFF_Y, y);
      dst.putDouble(base + ODOM_OFF_THETA, theta);
      dst.putDouble(base + ODOM_OFF_VX, vx);
      dst.putDouble(base + ODOM_OFF_VY, vy);
      dst.putDouble(base + ODOM_OFF_OMEGA, omega);
    }

    /**
     * Writes a Stream 2 vision-pose packet into {@code dst} starting at {@code base}.
     *
     * <p>The controller never sends Stream 2 in production. This exists for loopback tests and for
     * any bench-side stand-in for the Orin.
     *
     * @param dst destination buffer, which must already be in {@link #ORDER}
     * @param base byte offset of the start of the packet within {@code dst}
     * @param seq monotonically increasing packet sequence number
     * @param timestamp the controller-originated timestamp, echoed verbatim
     * @param x field-relative X position, in meters
     * @param y field-relative Y position, in meters
     * @param theta field-relative heading, in radians
     */
    public static void encodeVisionPose(
        ByteBuffer dst, int base, long seq, double timestamp, double x, double y, double theta) {
      dst.putLong(base + POSE_OFF_SEQ, seq);
      dst.putDouble(base + POSE_OFF_TIMESTAMP, timestamp);
      dst.putDouble(base + POSE_OFF_X, x);
      dst.putDouble(base + POSE_OFF_Y, y);
      dst.putDouble(base + POSE_OFF_THETA, theta);
    }

    /**
     * Allocates an off-heap buffer already set to {@link #ORDER}.
     *
     * @param size buffer size in bytes
     * @return a direct {@link ByteBuffer} in little-endian order
     */
    public static ByteBuffer allocate(int size) {
      return ByteBuffer.allocateDirect(size).order(ORDER);
    }

    private VisionLinkProtocol() {}
  }

  // ===========================================================================================
  // Counter
  // ===========================================================================================

  /**
   * A lock-free, allocation-free event counter with exactly one writing thread.
   *
   * <p>Only the owning thread may call {@link #increment()}; any thread may call {@link #get()}.
   * Opaque access gives atomic 64-bit reads and writes even on the roboRIO's 32-bit ARM JVM, and
   * avoids the CAS/locked-add that {@code AtomicLong.incrementAndGet} would put on every hot path.
   */
  public static final class Counter {
    private static final VarHandle VALUE;

    static {
      try {
        VALUE = MethodHandles.lookup().findVarHandle(Counter.class, "m_value", long.class);
      } catch (ReflectiveOperationException e) {
        throw new ExceptionInInitializerError(e);
      }
    }

    @SuppressWarnings("unused") // accessed through VALUE
    private long m_value;

    /** Adds one. Owning thread only. */
    public void increment() {
      VALUE.setOpaque(this, (long) VALUE.getOpaque(this) + 1L);
    }

    /**
     * Returns the current count. Safe from any thread.
     *
     * @return the count
     */
    public long get() {
      return (long) VALUE.getOpaque(this);
    }
  }

  // ===========================================================================================
  // StatsPublisher
  // ===========================================================================================

  /**
   * Low-rate NetworkTables publisher for one thread's counters.
   *
   * <p>The protocol requires every dropped packet to be visible somewhere rather than silently
   * discarded. Each owning thread holds one of these and, when {@link #due()} says so, fills {@link
   * #values()} and calls {@link #publish()}. The value array is preallocated and {@code
   * IntegerEntry.set(long)} does not allocate, so publishing adds no garbage.
   *
   * <p>Alongside the counters it publishes the owning thread's cumulative allocated bytes (where the
   * JVM supports it), so the zero-allocation claim can be watched on the robot: after warm-up the
   * value should stop climbing.
   *
   * <p>This is telemetry <i>about</i> the link &mdash; it is not the link. The data itself never
   * touches NetworkTables; NT4's update coalescing is exactly what these raw UDP streams avoid.
   *
   * <p>NT initialization is deferred to the first publish and guarded: on a desktop JVM without
   * ntcore loaded, publishing disables itself once and the caller's loop carries on unaffected.
   */
  public static final class StatsPublisher {
    private final String m_tableName;
    private final String m_allocName;
    private final String[] m_fieldNames;
    private final long[] m_values;
    private final double m_periodSeconds;

    private IntegerEntry[] m_entries;
    private IntegerEntry m_allocEntry;
    private boolean m_disabled;
    private double m_nextPublishSeconds = Double.NEGATIVE_INFINITY;

    /**
     * @param tableName NetworkTables table to publish under
     * @param periodSeconds minimum time between publishes
     * @param allocName topic for the owning thread's allocated-bytes gauge
     * @param fieldNames counter topic names, positionally matched to {@link #values()}
     */
    public StatsPublisher(
        String tableName, double periodSeconds, String allocName, String... fieldNames) {
      m_tableName = tableName;
      m_periodSeconds = periodSeconds;
      m_allocName = allocName;
      m_fieldNames = fieldNames.clone();
      m_values = new long[fieldNames.length];
    }

    /**
     * Returns true, and schedules the next publish, if a period has elapsed since the last one.
     *
     * @return whether the caller should fill {@link #values()} and call {@link #publish()} now
     */
    public boolean due() {
      if (m_disabled) {
        return false;
      }
      double now = SpineClock.now();
      if (now < m_nextPublishSeconds) {
        return false;
      }
      m_nextPublishSeconds = now + m_periodSeconds;
      return true;
    }

    /**
     * Returns the preallocated value array, positionally matched to the field names.
     *
     * @return the array to fill before {@link #publish()}
     */
    public long[] values() {
      return m_values;
    }

    /** Publishes {@link #values()} and the owning thread's allocated-bytes gauge. */
    public void publish() {
      try {
        if (m_entries == null) {
          NetworkTable table = NetworkTableInstance.getDefault().getTable(m_tableName);
          m_entries = new IntegerEntry[m_fieldNames.length];
          for (int i = 0; i < m_fieldNames.length; i++) {
            m_entries[i] = table.getIntegerTopic(m_fieldNames[i]).getEntry(0);
          }
          m_allocEntry = table.getIntegerTopic(m_allocName).getEntry(-1);
        }
        for (int i = 0; i < m_entries.length; i++) {
          m_entries[i].set(m_values[i]);
        }
        m_allocEntry.set(AllocGauge.currentThreadAllocatedBytes());
      } catch (Throwable t) {
        // No ntcore (headless test JVM), or NT is otherwise unavailable. Counters are still
        // readable through each stream's getters; don't let telemetry take down the caller's loop.
        m_disabled = true;
        System.err.println(
            "[VisionLink] NetworkTables counters unavailable for "
                + m_tableName
                + ": "
                + t
                + " (counters remain readable via getters)");
      }
    }
  }

  /**
   * Reads the calling thread's cumulative allocated bytes via {@code com.sun.management}, isolated
   * in its own class so a JRE without {@code jdk.management} only disables the gauge.
   */
  static final class AllocGauge {
    private static final com.sun.management.ThreadMXBean s_bean;

    static {
      com.sun.management.ThreadMXBean bean = null;
      try {
        bean = (com.sun.management.ThreadMXBean) java.lang.management.ManagementFactory.getThreadMXBean();
        if (!bean.isThreadAllocatedMemorySupported()) {
          bean = null;
        } else if (!bean.isThreadAllocatedMemoryEnabled()) {
          bean.setThreadAllocatedMemoryEnabled(true);
        }
      } catch (Throwable t) {
        bean = null;
      }
      s_bean = bean;
    }

    /**
     * Returns the calling thread's cumulative allocated bytes.
     *
     * @return allocated bytes, or -1 where the JVM does not support the measurement
     */
    static long currentThreadAllocatedBytes() {
      return s_bean == null ? -1 : s_bean.getCurrentThreadAllocatedBytes();
    }

    private AllocGauge() {}
  }

  // ===========================================================================================
  // HalPeriodicThread
  // ===========================================================================================

  /**
   * A dedicated Java thread woken on an absolute schedule by a HAL notifier alarm.
   *
   * <p>This talks to {@link NotifierJNI} directly instead of going through {@code
   * edu.wpi.first.wpilibj.Notifier}, whose callback dispatch takes a {@code ReentrantLock} on every
   * tick. The loop here takes no lock and allocates nothing: set the alarm, block in {@code
   * waitForNotifierAlarm}, run the tick, advance the deadline.
   *
   * <p>Deadlines come from {@link SpineClock#nowMicros()} &mdash; the same timebase the HAL alarm
   * fires on. If a tick overruns so the next deadline has already passed, the loop resyncs to
   * now plus one period instead of bursting to catch up, and counts a late tick. A burst would
   * distort the very timing the Orin depends on.
   *
   * <p>A tick that throws is counted and the loop keeps running. Real-time priority is opt-in:
   * measure {@link #getLateTickCount()} first.
   */
  public static final class HalPeriodicThread {
    private final String m_name;
    private final long m_periodMicros;
    private final Runnable m_tick;
    private final int m_realtimePriority;

    private final Counter m_lateTicks = new Counter();
    private final Counter m_tickErrors = new Counter();

    private volatile boolean m_running;
    private volatile int m_handle;
    private Thread m_thread;

    /**
     * @param name thread and notifier name
     * @param periodSeconds tick period
     * @param tick work to run each period; runs on this thread only
     * @param realtimePriority SCHED_FIFO priority (1-99) for the thread, or 0 to leave it at normal
     *     priority
     */
    public HalPeriodicThread(String name, double periodSeconds, Runnable tick, int realtimePriority) {
      if (!(periodSeconds > 0)) {
        throw new IllegalArgumentException("period must be positive, got " + periodSeconds);
      }
      m_name = name;
      m_periodMicros = Math.max(1L, Math.round(periodSeconds * 1e6));
      m_tick = tick;
      m_realtimePriority = realtimePriority;
    }

    /** Starts the thread. Idempotent. */
    public synchronized void start() {
      if (m_thread != null) {
        return;
      }
      int handle = NotifierJNI.initializeNotifier();
      NotifierJNI.setNotifierName(handle, m_name);
      m_handle = handle;
      m_running = true;
      m_thread = new Thread(this::run, m_name);
      m_thread.setDaemon(true);
      m_thread.start();
    }

    private void run() {
      if (m_realtimePriority > 0) {
        Threads.setCurrentThreadPriority(true, m_realtimePriority);
      }
      final int handle = m_handle;
      long deadline = SpineClock.nowMicros() + m_periodMicros;
      try {
        while (m_running) {
          NotifierJNI.updateNotifierAlarm(handle, deadline);
          if (NotifierJNI.waitForNotifierAlarm(handle) == 0) {
            break; // notifier stopped
          }
          try {
            m_tick.run();
          } catch (Throwable t) {
            if (m_tickErrors.get() == 0) {
              System.err.println("[VisionLink] " + m_name + " tick threw: " + t);
            }
            m_tickErrors.increment();
          }
          deadline += m_periodMicros;
          long now = SpineClock.nowMicros();
          if (deadline <= now) {
            m_lateTicks.increment();
            deadline = now + m_periodMicros;
          }
        }
      } finally {
        NotifierJNI.cleanNotifier(handle);
      }
    }

    /** Stops the thread and waits briefly for it to exit. */
    public void stop() {
      Thread t;
      synchronized (this) {
        m_running = false;
        t = m_thread;
      }
      if (t == null) {
        return;
      }
      NotifierJNI.stopNotifier(m_handle);
      try {
        t.join(1000);
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
      }
    }

    /**
     * Returns the number of ticks after which the schedule had to be resynced.
     *
     * @return the late-tick count
     */
    public long getLateTickCount() {
      return m_lateTicks.get();
    }

    /**
     * Returns the number of ticks that threw.
     *
     * @return the tick-error count
     */
    public long getTickErrorCount() {
      return m_tickErrors.get();
    }

    /**
     * Returns the underlying thread, or null before {@link #start()}.
     *
     * @return the thread
     */
    public synchronized Thread getThread() {
      return m_thread;
    }
  }

  // ===========================================================================================
  // OdometrySample
  // ===========================================================================================

  /**
   * Mutable carrier for one Stream 1 odometry sample, all primitives.
   *
   * <p>{@link OdometrySender} keeps exactly one instance and refills it every tick, so nothing is
   * allocated in the steady state. Only ever touched by the odometry thread, which is why the fields
   * carry no synchronization.
   */
  public static final class OdometrySample {
    /** Controller timestamp at which the sensors were read, in seconds. */
    public double timestamp;

    /** X position in the odometry frame, in meters. */
    public double x;

    /** Y position in the odometry frame, in meters. */
    public double y;

    /** Heading, in radians. */
    public double theta;

    /** Field-frame X velocity, in meters per second. */
    public double vx;

    /** Field-frame Y velocity, in meters per second. */
    public double vy;

    /** Angular velocity, in radians per second. */
    public double omega;

    /**
     * Sets every field in one call.
     *
     * @param timestamp controller timestamp, in seconds
     * @param x X position, in meters
     * @param y Y position, in meters
     * @param theta heading, in radians
     * @param vx field-frame X velocity, in meters per second
     * @param vy field-frame Y velocity, in meters per second
     * @param omega angular velocity, in radians per second
     */
    public void set(
        double timestamp, double x, double y, double theta, double vx, double vy, double omega) {
      this.timestamp = timestamp;
      this.x = x;
      this.y = y;
      this.theta = theta;
      this.vx = vx;
      this.vy = vy;
      this.omega = omega;
    }
  }

  // ===========================================================================================
  // OdometrySource
  // ===========================================================================================

  /**
   * Supplies swerve odometry to {@link OdometrySender} for Stream 1.
   *
   * <p>Called on the odometry thread and only there, so an implementation may keep thread-confined
   * state without synchronization. It must fill the caller-owned sample rather than allocate, must
   * not block, and must take its timestamp from {@link SpineClock#now()}.
   */
  public interface OdometrySource {
    /**
     * Fills {@code dest} with the current odometry state.
     *
     * @param dest sample to overwrite; reused across calls, so every field must be written
     * @return true if {@code dest} holds a valid sample; false to skip this tick and send nothing
     */
    boolean fill(OdometrySample dest);

    /**
     * Returns how many ticks saw no sensor input change since the previous tick, i.e. resent stale
     * data. Published alongside the Stream 1 counters.
     *
     * @return the stale-tick count, or 0 if the source doesn't track it
     */
    default long getStaleTickCount() {
      return 0;
    }
  }

  // ===========================================================================================
  // TriggerRing
  // ===========================================================================================

  /**
   * The controller-local table of recent camera triggers that backs Stream 3, handed between
   * threads through a seqlock.
   *
   * <p>Holds the last {@link VisionLinkProtocol#TRIG_ENTRY_COUNT} {@code triggerSeq -> timestamp}
   * pairs. Every outgoing packet carries all five entries, most recent first, so a single dropped
   * datagram never loses a timestamp.
   *
   * <p>Slots not yet filled are serialized as {@code triggerSeq = }{@value #EMPTY_TRIGGER_SEQ} with a
   * zero timestamp. Real sequence numbers start at 0, so the Orin's dedup rule &mdash; accept only
   * entries whose {@code triggerSeq} exceeds the highest seen so far &mdash; rejects them naturally.
   *
   * <h2>Concurrency: single-writer seqlock</h2>
   *
   * <p>{@link #record} must only ever be called from <b>one</b> thread (the trigger pulser). It bumps
   * the version to odd, writes the slot with plain stores, and releases the version back to even
   * &mdash; wait-free, no allocation. {@link #writeInto} (the Stream 3 sender) reads the version with
   * acquire semantics, copies the table, fences, and retries if the version moved or was odd. This is
   * the same optimistic-read protocol as {@code StampedLock.tryOptimisticRead}. A retry needs a
   * trigger to land during a copy of a few dozen bytes, which at 60 Hz is rare; retries are counted.
   *
   * <p>Kept free of WPILib imports and of any socket so the logic is directly unit-testable.
   */
  public static final class TriggerRing {
    /** Sentinel {@code triggerSeq} written for slots that have not been filled yet. */
    public static final long EMPTY_TRIGGER_SEQ = -1L;

    private static final int kCapacity = VisionLinkProtocol.TRIG_ENTRY_COUNT;

    private static final VarHandle VERSION;
    private static final VarHandle NEXT_SEQ;

    static {
      try {
        MethodHandles.Lookup lookup = MethodHandles.lookup();
        VERSION = lookup.findVarHandle(TriggerRing.class, "m_version", long.class);
        NEXT_SEQ = lookup.findVarHandle(TriggerRing.class, "m_nextTriggerSeq", long.class);
      } catch (ReflectiveOperationException e) {
        throw new ExceptionInInitializerError(e);
      }
    }

    private final long[] m_seqs = new long[kCapacity];
    private final double[] m_timestamps = new double[kCapacity];

    // Written only by the single writer; read racily by the reader inside the seqlock.
    private int m_head; // index of the next slot to write
    private int m_filled; // valid slots, saturating at kCapacity

    @SuppressWarnings("unused") // accessed through VERSION
    private long m_version; // even: stable; odd: write in progress

    @SuppressWarnings("unused") // accessed through NEXT_SEQ
    private long m_nextTriggerSeq;

    /**
     * Records one camera trigger, assigning it the next sequence number. Single writer thread only.
     *
     * @param timestamp the controller timestamp of the trigger pulse, in seconds
     * @return the {@code triggerSeq} assigned to this trigger
     */
    public long record(double timestamp) {
      long v = (long) VERSION.getOpaque(this);
      VERSION.setOpaque(this, v + 1); // odd: write in progress
      VarHandle.storeStoreFence(); // odd version is visible before any slot write

      long seq = (long) NEXT_SEQ.getOpaque(this);
      int head = m_head;
      m_seqs[head] = seq;
      m_timestamps[head] = timestamp;
      m_head = head + 1 == kCapacity ? 0 : head + 1;
      if (m_filled < kCapacity) {
        m_filled++;
      }
      NEXT_SEQ.setOpaque(this, seq + 1);

      VERSION.setRelease(this, v + 2); // even: slot writes are visible before this
      return seq;
    }

    /**
     * Serializes a consistent snapshot of the table into a Stream 3 packet, most recent entry first.
     *
     * @param dst destination buffer, which must already be in {@link VisionLinkProtocol#ORDER}
     * @param base byte offset of the start of the packet within {@code dst}
     * @param packetSeq the packet's own sequence number, independent of the trigger sequence numbers
     * @return how many times the read had to be retried because a write overlapped it
     */
    public int writeInto(ByteBuffer dst, int base, long packetSeq) {
      dst.putLong(base + VisionLinkProtocol.TRIG_OFF_PACKET_SEQ, packetSeq);

      int retries = 0;
      while (true) {
        long v1 = (long) VERSION.getAcquire(this);
        if ((v1 & 1L) != 0) {
          retries++;
          Thread.onSpinWait();
          continue;
        }

        int head = m_head;
        int filled = m_filled;
        for (int i = 0; i < kCapacity; i++) {
          // Entry 0 is the most recently recorded trigger, which sits one slot behind the head.
          int slot = Math.floorMod(head - 1 - i, kCapacity);
          boolean valid = i < filled;
          dst.putLong(
              base + VisionLinkProtocol.trigOffTriggerSeq(i),
              valid ? m_seqs[slot] : EMPTY_TRIGGER_SEQ);
          dst.putDouble(
              base + VisionLinkProtocol.trigOffTimestamp(i), valid ? m_timestamps[slot] : 0.0);
        }

        VarHandle.loadLoadFence(); // the copy above completes before re-reading the version
        if ((long) VERSION.getOpaque(this) == v1) {
          return retries;
        }
        retries++;
      }
    }

    /**
     * Returns the sequence number that the next recorded trigger will receive. Safe from any
     * thread.
     *
     * @return the next {@code triggerSeq}
     */
    public long getNextTriggerSeq() {
      return (long) NEXT_SEQ.getOpaque(this);
    }
  }

  // ===========================================================================================
  // OdometrySender
  // ===========================================================================================

  /**
   * Stream 1 sender: swerve odometry, controller to Orin, 64 bytes per packet.
   *
   * <p>Runs on its own {@link HalPeriodicThread} &mdash; never on {@code robotPeriodic()}. A previous
   * test rig on this project traced sparse 10-12 ms latency spikes directly to a high-rate send loop
   * sharing the main robot thread, so this separation is a hard requirement.
   *
   * <p>The source is sampled and the packet is sent on the same thread in the same tick, so odometry
   * never crosses a thread boundary and there is no handoff to get wrong. One off-heap buffer and one
   * {@link OdometrySample} are allocated at construction and reused forever.
   *
   * <p>Nothing starts until {@link #start} is called; constructing the sender only opens the socket.
   */
  public static final class OdometrySender implements AutoCloseable {
    private final InetSocketAddress m_target;
    private final DatagramChannel m_channel;
    private final double m_periodSeconds;

    private final ByteBuffer m_buf = VisionLinkProtocol.allocate(VisionLinkProtocol.ODOM_SIZE);
    private final OdometrySample m_sample = new OdometrySample();

    private final Counter m_sentCount = new Counter();
    private final Counter m_sendErrorCount = new Counter();
    private final Counter m_skippedCount = new Counter();
    private final StatsPublisher m_stats;

    private long m_seq; // odometry thread only
    private volatile boolean m_closed;
    private OdometrySource m_source;
    private HalPeriodicThread m_periodic;

    /**
     * Creates a sender from the link configuration.
     *
     * @param config link configuration
     */
    public OdometrySender(Config config) {
      this(
          config.orinHost(),
          config.odometryPort(),
          config.odometryRateHz(),
          config.statsTable(),
          config.statsPublishPeriodSeconds());
    }

    /**
     * Creates a sender against an explicit destination, e.g. for loopback tests.
     *
     * @param host destination host
     * @param port destination UDP port
     * @param rateHz send rate, in hertz
     * @param statsTable NetworkTables table for the counters
     * @param statsPeriodSeconds counter publish period
     */
    public OdometrySender(
        String host, int port, int rateHz, String statsTable, double statsPeriodSeconds) {
      m_target = new InetSocketAddress(host, port);
      m_periodSeconds = 1.0 / rateHz;
      m_stats =
          new StatsPublisher(
              statsTable,
              statsPeriodSeconds,
              "odomThreadAllocBytes",
              "odomSent",
              "odomSendErrors",
              "odomSkipped",
              "odomLateTicks",
              "odomTickErrors",
              "odomStaleTicks");
      try {
        m_channel = DatagramChannel.open();
        m_channel.configureBlocking(true);
      } catch (IOException e) {
        throw new RuntimeException("[VisionLink] Failed to open odometry sender socket", e);
      }
    }

    /**
     * Starts the send loop. Idempotent; a second call is ignored.
     *
     * @param source supplies odometry each tick, on the odometry thread
     */
    public synchronized void start(OdometrySource source) {
      if (m_periodic != null) {
        return;
      }
      m_source = source;
      m_periodic = new HalPeriodicThread("VisionLink-Odometry", m_periodSeconds, this::tick, 0);
      m_periodic.start();
      System.out.println("[VisionLink] Odometry sender started -> " + m_target);
    }

    private void tick() {
      if (m_closed) {
        return;
      }
      if (m_source.fill(m_sample)) {
        VisionLinkProtocol.encodeOdometry(
            m_buf,
            0,
            m_seq++,
            m_sample.timestamp,
            m_sample.x,
            m_sample.y,
            m_sample.theta,
            m_sample.vx,
            m_sample.vy,
            m_sample.omega);
        try {
          m_buf.clear(); // absolute puts left position alone; rewind for the relative send
          m_channel.send(m_buf, m_target);
          m_sentCount.increment();
        } catch (ClosedChannelException e) {
          m_closed = true; // shutting down
        } catch (IOException e) {
          m_sendErrorCount.increment();
        }
      } else {
        m_skippedCount.increment();
      }

      if (m_stats.due()) {
        long[] v = m_stats.values();
        v[0] = m_sentCount.get();
        v[1] = m_sendErrorCount.get();
        v[2] = m_skippedCount.get();
        v[3] = m_periodic.getLateTickCount();
        v[4] = m_periodic.getTickErrorCount();
        v[5] = m_source.getStaleTickCount();
        m_stats.publish();
      }
    }

    /**
     * Returns the number of odometry packets successfully handed to the socket.
     *
     * @return the sent-packet count
     */
    public long getSentCount() {
      return m_sentCount.get();
    }

    /**
     * Returns the number of ticks whose send threw an {@link IOException}.
     *
     * @return the send-error count
     */
    public long getSendErrorCount() {
      return m_sendErrorCount.get();
    }

    /**
     * Returns the number of ticks the source declined to supply a sample for.
     *
     * @return the skipped-tick count
     */
    public long getSkippedCount() {
      return m_skippedCount.get();
    }

    /**
     * Returns the periodic thread driving this sender, or null before {@link #start}.
     *
     * @return the periodic thread
     */
    public synchronized HalPeriodicThread getPeriodicThread() {
      return m_periodic;
    }

    @Override
    public void close() {
      m_closed = true;
      HalPeriodicThread periodic = getPeriodicThread();
      if (periodic != null) {
        periodic.stop();
      }
      try {
        m_channel.close();
      } catch (IOException e) {
        System.err.println("[VisionLink] Error closing odometry sender: " + e.getMessage());
      }
    }
  }

  // ===========================================================================================
  // TriggerTableSender
  // ===========================================================================================

  /**
   * Stream 3 sender: camera trigger table, controller to Orin, 88 bytes per packet.
   *
   * <p>Fires alongside each hardware camera trigger pulse rather than at a fixed rate. Each packet
   * carries the five most recent {@code triggerSeq -> timestamp} pairs (see {@link TriggerRing}), so
   * a single dropped datagram costs nothing.
   *
   * <p>{@link #recordTrigger} does only two cheap, lock-free things on the caller's thread: write the
   * seqlocked table slot, and unpark the dedicated sender thread. The socket write always happens on
   * that sender thread. {@code recordTrigger} must be called from a single thread.
   *
   * <p>On the Orin side, the receiver keeps a {@code lastSeenTriggerSeq} and forwards only entries
   * newer than it. Each forwarded timestamp travels unchanged with the frame that trigger exposed,
   * and comes back verbatim in the Stream 2 pose computed from that frame (see {@code
   * docs/spine-link.md}, section 3).
   *
   * <p>Nothing starts until {@link #start} is called; constructing the sender only opens the socket.
   */
  public static final class TriggerTableSender implements AutoCloseable {
    private final InetSocketAddress m_target;
    private final DatagramChannel m_channel;

    private final TriggerRing m_ring = new TriggerRing();
    private final ByteBuffer m_buf = VisionLinkProtocol.allocate(VisionLinkProtocol.TRIG_SIZE);

    // Sender-thread counters.
    private final Counter m_sentCount = new Counter();
    private final Counter m_sendErrorCount = new Counter();
    private final Counter m_seqlockRetries = new Counter();
    // Trigger-thread counter.
    private final Counter m_triggerCount = new Counter();

    private final AtomicBoolean m_running = new AtomicBoolean(false);
    private final AtomicBoolean m_pending = new AtomicBoolean(false);
    private final StatsPublisher m_stats;

    private long m_packetSeq; // sender thread only
    private volatile Thread m_thread;

    /**
     * Creates a sender from the link configuration.
     *
     * @param config link configuration
     */
    public TriggerTableSender(Config config) {
      this(
          config.orinHost(),
          config.triggerTablePort(),
          config.statsTable(),
          config.statsPublishPeriodSeconds());
    }

    /**
     * Creates a sender against an explicit destination, e.g. for loopback tests.
     *
     * @param host destination host
     * @param port destination UDP port
     * @param statsTable NetworkTables table for the counters
     * @param statsPeriodSeconds counter publish period
     */
    public TriggerTableSender(String host, int port, String statsTable, double statsPeriodSeconds) {
      m_target = new InetSocketAddress(host, port);
      m_stats =
          new StatsPublisher(
              statsTable,
              statsPeriodSeconds,
              "trigSendThreadAllocBytes",
              "trigSent",
              "trigSendErrors",
              "trigRecorded",
              "trigSeqlockRetries");
      try {
        m_channel = DatagramChannel.open();
        m_channel.configureBlocking(true);
      } catch (IOException e) {
        throw new RuntimeException("[VisionLink] Failed to open trigger table sender socket", e);
      }
    }

    /** Starts the sender thread. Idempotent; a second call is ignored. */
    public synchronized void start() {
      if (!m_running.compareAndSet(false, true)) {
        return;
      }
      Thread t = new Thread(this::sendLoop, "VisionLink-TriggerTableSend");
      t.setDaemon(true);
      m_thread = t;
      t.start();
      System.out.println("[VisionLink] Trigger table sender started -> " + m_target);
    }

    /**
     * Records a camera trigger at the current controller timestamp and queues a packet.
     *
     * @return the {@code triggerSeq} assigned to this trigger
     */
    public long recordTrigger() {
      return recordTrigger(SpineClock.now());
    }

    /**
     * Records a camera trigger at an explicit controller timestamp and queues a packet. Must be
     * called from a single thread; never touches the socket.
     *
     * @param timestamp the controller timestamp of the trigger pulse, in seconds
     * @return the {@code triggerSeq} assigned to this trigger
     */
    public long recordTrigger(double timestamp) {
      long seq = m_ring.record(timestamp);
      m_triggerCount.increment();

      // Set the flag before unparking. If the sender is mid-loop and misses the park, the unpark
      // permit makes its next park() return immediately and it picks the flag up there.
      m_pending.set(true);
      Thread t = m_thread;
      if (t != null) {
        LockSupport.unpark(t);
      }
      return seq;
    }

    private void sendLoop() {
      while (m_running.get()) {
        if (m_pending.compareAndSet(true, false)) {
          int retries = m_ring.writeInto(m_buf, 0, m_packetSeq++);
          for (int i = 0; i < retries; i++) {
            m_seqlockRetries.increment();
          }
          try {
            m_buf.clear(); // absolute puts left position alone; rewind for the relative send
            m_channel.send(m_buf, m_target);
            m_sentCount.increment();
          } catch (ClosedChannelException e) {
            break; // shutting down
          } catch (IOException e) {
            m_sendErrorCount.increment();
          }
          if (m_stats.due()) {
            long[] v = m_stats.values();
            v[0] = m_sentCount.get();
            v[1] = m_sendErrorCount.get();
            v[2] = m_triggerCount.get();
            v[3] = m_seqlockRetries.get();
            m_stats.publish();
          }
        } else {
          LockSupport.park(this);
        }
      }
    }

    /**
     * Returns the controller-local trigger table, exposed for inspection and testing.
     *
     * @return the backing trigger ring
     */
    public TriggerRing getRing() {
      return m_ring;
    }

    /**
     * Returns the number of trigger-table packets successfully handed to the socket.
     *
     * @return the sent-packet count
     */
    public long getSentCount() {
      return m_sentCount.get();
    }

    /**
     * Returns the number of sends that threw an {@link IOException}.
     *
     * @return the send-error count
     */
    public long getSendErrorCount() {
      return m_sendErrorCount.get();
    }

    /**
     * Returns the number of triggers recorded into the local table.
     *
     * <p>May exceed {@link #getSentCount} when triggers arrive faster than the sender drains them;
     * that is harmless, because the newer packet already carries the older entries.
     *
     * @return the recorded-trigger count
     */
    public long getTriggerCount() {
      return m_triggerCount.get();
    }

    /**
     * Returns how many seqlock read retries the sender has needed.
     *
     * @return the retry count
     */
    public long getSeqlockRetryCount() {
      return m_seqlockRetries.get();
    }

    /**
     * Returns the sender thread, or null before {@link #start}.
     *
     * @return the sender thread
     */
    public Thread getThread() {
      return m_thread;
    }

    @Override
    public void close() {
      m_running.set(false);
      Thread t = m_thread;
      if (t != null) {
        LockSupport.unpark(t); // wake it so it can observe m_running
      }
      try {
        m_channel.close();
      } catch (IOException e) {
        System.err.println("[VisionLink] Error closing trigger table sender: " + e.getMessage());
      }
    }
  }

  // ===========================================================================================
  // VisionPoseReceiver
  // ===========================================================================================

  /**
   * Stream 2 receiver: fused vision poses, Orin to controller, 40 bytes per packet, event-driven.
   *
   * <p>Poses arrive whenever the Orin's pose fusion produces one, so a dedicated daemon thread sits
   * in a blocking {@code receive()} and hands packets to the consumer through a lock-free
   * single-producer/single-consumer ring: volatile {@code head} and {@code tail}, power-of-two
   * capacity, {@code & MASK} instead of modulo, no locks and no per-packet objects anywhere on the
   * path.
   *
   * <p>Both ring and snapshot are off-heap {@link ByteBuffer}s allocated once at construction. The
   * network thread writes only the ring, the consumer thread reads only the snapshot, and {@link
   * #readPackets} moves bytes between them in bulk. {@link PoseView} is a flyweight cursor over the
   * snapshot rather than an object per packet.
   *
   * <p><b>Every drop is counted.</b> A wrong-size datagram increments {@link
   * #getDroppedSizeMismatch}; a pose whose timestamp is NaN, in the future, or older than the
   * configured max age &mdash; or whose pose fields are not finite &mdash; increments {@link
   * #getDroppedStale}; a full ring increments {@link #getDroppedRingFull}. All are published to
   * NetworkTables and readable through getters.
   *
   * <p>The {@code timestamp} field is the controller-originated timestamp echoed back verbatim by the
   * Orin. {@link PoseView#getTimestamp} returns exactly the bits that arrived; this class never
   * recomputes, offsets or clock-corrects it.
   *
   * <p>Typical consumer use, from the robot loop, with the consumer held in a field so no lambda is
   * allocated per loop:
   *
   * <pre>
   * if (receiver.readPackets() &gt; 0) {
   *   receiver.forEachPacket(m_applyVision);
   * }
   * </pre>
   */
  public static final class VisionPoseReceiver implements AutoCloseable {
    private static final int kPacketSize = VisionLinkProtocol.POSE_SIZE;

    private final int m_capacity;
    private final int m_mask;
    private final double m_maxAgeSeconds;

    /** Written only by the network thread. */
    private final ByteBuffer m_ringBuffer;

    /** Read only by the consumer thread. */
    private final ByteBuffer m_snapshotBuffer;

    private final ByteBuffer m_socketBuffer;
    private final DatagramChannel m_channel;
    private final PoseView m_view = new PoseView();

    private volatile int m_head; // next slot the network thread will write
    private volatile int m_tail; // next slot the consumer will read
    private int m_snapshotCount;

    // Network-thread counters.
    private final Counter m_recvCount = new Counter();
    private final Counter m_droppedSizeMismatch = new Counter();
    private final Counter m_droppedStale = new Counter();
    private final Counter m_droppedRingFull = new Counter();

    private final AtomicBoolean m_running = new AtomicBoolean(false);
    private final StatsPublisher m_stats;

    private Thread m_thread;

    /**
     * Creates a receiver from the link configuration.
     *
     * @param config link configuration
     */
    public VisionPoseReceiver(Config config) {
      this(
          config.visionPosePort(),
          config.visionRingSize(),
          config.visionSocketRcvBufBytes(),
          config.visionMaxAgeSeconds(),
          config.statsTable(),
          config.statsPublishPeriodSeconds());
    }

    /**
     * Creates a receiver on an explicit port, e.g. for loopback tests.
     *
     * @param port UDP port to bind, or 0 for an ephemeral port
     * @param ringSize ring capacity in packets; must be a power of two
     * @param rcvBufBytes socket receive buffer size
     * @param maxAgeSeconds poses older than this are dropped
     * @param statsTable NetworkTables table for the counters
     * @param statsPeriodSeconds counter publish period
     */
    public VisionPoseReceiver(
        int port,
        int ringSize,
        int rcvBufBytes,
        double maxAgeSeconds,
        String statsTable,
        double statsPeriodSeconds) {
      if (ringSize < 2 || Integer.bitCount(ringSize) != 1) {
        throw new IllegalArgumentException("ring size must be a power of two, got " + ringSize);
      }
      m_capacity = ringSize;
      m_mask = ringSize - 1;
      m_maxAgeSeconds = maxAgeSeconds;

      m_ringBuffer = VisionLinkProtocol.allocate(ringSize * kPacketSize);
      m_snapshotBuffer = VisionLinkProtocol.allocate(ringSize * kPacketSize);

      // One byte larger than a valid packet so an oversized datagram is detectable rather than
      // being silently truncated to exactly kPacketSize and passing the size check.
      m_socketBuffer = VisionLinkProtocol.allocate(kPacketSize + 1);

      m_stats =
          new StatsPublisher(
              statsTable,
              statsPeriodSeconds,
              "poseRecvThreadAllocBytes",
              "poseRecvd",
              "poseDroppedSize",
              "poseDroppedStale",
              "poseDroppedRingFull");

      try {
        m_channel = DatagramChannel.open();
        m_channel.bind(new InetSocketAddress(port));
        m_channel.setOption(StandardSocketOptions.SO_RCVBUF, rcvBufBytes);
        m_channel.configureBlocking(true);
      } catch (IOException e) {
        throw new RuntimeException("[VisionLink] Failed to bind vision pose receiver", e);
      }
    }

    /** Starts the receive loop. Idempotent; a second call is ignored. */
    public synchronized void start() {
      if (!m_running.compareAndSet(false, true)) {
        return;
      }
      m_thread = new Thread(this::receiveLoop, "VisionLink-VisionPoseRecv");
      m_thread.setDaemon(true);
      m_thread.start();

      System.out.println("[VisionLink] Vision pose receiver started on port " + getBoundPort());
    }

    private void receiveLoop() {
      while (m_running.get()) {
        try {
          m_socketBuffer.clear();
          m_channel.receive(m_socketBuffer);
          m_socketBuffer.flip();
          accept();
          publishStatsIfDue();
        } catch (ClosedChannelException e) {
          break; // shutting down
        } catch (IOException e) {
          if (m_running.get()) {
            System.err.println("[VisionLink] Vision pose receive error: " + e.getMessage());
          }
        }
      }
    }

    /** Validates the datagram in the socket buffer and moves it into the ring. */
    private void accept() {
      if (m_socketBuffer.remaining() != kPacketSize) {
        m_droppedSizeMismatch.increment();
        return;
      }

      double timestamp = m_socketBuffer.getDouble(VisionLinkProtocol.POSE_OFF_TIMESTAMP);
      double now = SpineClock.now();
      // !(a <= b) is also true for NaN, so this rejects NaN and future timestamps in one test.
      if (!(timestamp <= now)
          || now - timestamp > m_maxAgeSeconds
          || !Double.isFinite(m_socketBuffer.getDouble(VisionLinkProtocol.POSE_OFF_X))
          || !Double.isFinite(m_socketBuffer.getDouble(VisionLinkProtocol.POSE_OFF_Y))
          || !Double.isFinite(m_socketBuffer.getDouble(VisionLinkProtocol.POSE_OFF_THETA))) {
        m_droppedStale.increment();
        return;
      }

      int h = m_head;
      int nextH = (h + 1) & m_mask;
      if (nextH == m_tail) {
        m_droppedRingFull.increment();
        return;
      }

      // Bulk copy, not a byte-at-a-time loop.
      m_ringBuffer.put(h * kPacketSize, m_socketBuffer, 0, kPacketSize);
      m_head = nextH; // publishes the slot to the consumer
      m_recvCount.increment();
    }

    private void publishStatsIfDue() {
      if (m_stats.due()) {
        long[] v = m_stats.values();
        v[0] = m_recvCount.get();
        v[1] = m_droppedSizeMismatch.get();
        v[2] = m_droppedStale.get();
        v[3] = m_droppedRingFull.get();
        m_stats.publish();
      }
    }

    /**
     * Drains everything received since the last call into the consumer-side snapshot.
     *
     * <p>Call this once per loop from the consumer thread, then iterate with {@link #forEachPacket}.
     *
     * @return the number of packets now available to iterate
     */
    public int readPackets() {
      int h = m_head;
      int t = m_tail;
      int count = 0;

      while (t != h && count < m_capacity) {
        // Bulk copy, not a byte-at-a-time loop.
        m_snapshotBuffer.put(count * kPacketSize, m_ringBuffer, t * kPacketSize, kPacketSize);
        t = (t + 1) & m_mask;
        count++;
      }

      m_tail = t; // frees the slots for the network thread
      m_snapshotCount = count;
      return count;
    }

    /**
     * Iterates the packets drained by the most recent {@link #readPackets}, oldest first.
     *
     * <p>The same {@link PoseView} instance is handed to every call, so do not retain it past the
     * callback; read what you need out of it instead. Pass a consumer held in a field, not a fresh
     * capturing lambda, to keep the call allocation-free.
     *
     * @param consumer receives the view and the packet's index within this batch
     */
    public void forEachPacket(ObjIntConsumer<PoseView> consumer) {
      for (int i = 0; i < m_snapshotCount; i++) {
        m_view.setIndex(i);
        consumer.accept(m_view, i);
      }
    }

    /**
     * Returns the local port the receiver is bound to, useful when constructed with port 0.
     *
     * @return the bound UDP port, or -1 if it cannot be determined
     */
    public int getBoundPort() {
      try {
        return ((InetSocketAddress) m_channel.getLocalAddress()).getPort();
      } catch (IOException e) {
        return -1;
      }
    }

    /**
     * Returns the number of well-formed pose packets accepted into the ring.
     *
     * @return the received-packet count
     */
    public long getRecvCount() {
      return m_recvCount.get();
    }

    /**
     * Returns the number of datagrams dropped because their length was not exactly 40 bytes.
     *
     * @return the size-mismatch drop count
     */
    public long getDroppedSizeMismatch() {
      return m_droppedSizeMismatch.get();
    }

    /**
     * Returns the number of packets dropped for a NaN, future or too-old timestamp, or a non-finite
     * pose.
     *
     * @return the stale/invalid drop count
     */
    public long getDroppedStale() {
      return m_droppedStale.get();
    }

    /**
     * Returns the number of packets dropped because the consumer had not drained the ring in time.
     *
     * @return the ring-full drop count
     */
    public long getDroppedRingFull() {
      return m_droppedRingFull.get();
    }

    /**
     * Returns the receiver thread, or null before {@link #start}.
     *
     * @return the receiver thread
     */
    public synchronized Thread getThread() {
      return m_thread;
    }

    @Override
    public void close() {
      m_running.set(false);
      try {
        m_channel.close(); // unblocks the receive() the thread is parked in
      } catch (IOException e) {
        System.err.println("[VisionLink] Error closing vision pose receiver: " + e.getMessage());
      }
    }

    /**
     * Zero-allocation flyweight view over one pose packet in the snapshot buffer.
     *
     * <p>Valid only inside a {@link #forEachPacket} callback; the cursor moves on the next iteration.
     */
    public final class PoseView {
      private int m_base;

      // Package-private: only the receiver may move the cursor.
      void setIndex(int index) {
        m_base = index * kPacketSize;
      }

      /**
       * Returns the Orin-assigned packet sequence number.
       *
       * @return the sequence number
       */
      public long getSeq() {
        return m_snapshotBuffer.getLong(m_base + VisionLinkProtocol.POSE_OFF_SEQ);
      }

      /**
       * Returns the controller timestamp of the camera trigger that produced this pose, in seconds.
       *
       * <p>Echoed back verbatim by the Orin from the Stream 3 trigger table and returned here
       * unmodified. Pass it straight to the pose estimator; do not adjust it.
       *
       * @return the controller-timebase timestamp
       */
      public double getTimestamp() {
        return m_snapshotBuffer.getDouble(m_base + VisionLinkProtocol.POSE_OFF_TIMESTAMP);
      }

      /**
       * Returns the field-relative X position, in meters.
       *
       * @return the X position
       */
      public double getX() {
        return m_snapshotBuffer.getDouble(m_base + VisionLinkProtocol.POSE_OFF_X);
      }

      /**
       * Returns the field-relative Y position, in meters.
       *
       * @return the Y position
       */
      public double getY() {
        return m_snapshotBuffer.getDouble(m_base + VisionLinkProtocol.POSE_OFF_Y);
      }

      /**
       * Returns the field-relative heading, in radians.
       *
       * @return the heading
       */
      public double getTheta() {
        return m_snapshotBuffer.getDouble(m_base + VisionLinkProtocol.POSE_OFF_THETA);
      }

      /**
       * Returns the pose as a {@link Pose2d}.
       *
       * <p>Unlike the accessors above, this allocates. It exists for the WPILib estimator API on the
       * main thread, at the event-driven Stream 2 rate.
       *
       * @return the pose
       */
      public Pose2d getPose2d() {
        return new Pose2d(getX(), getY(), Rotation2d.fromRadians(getTheta()));
      }
    }
  }

  private SpineUDP() {}
}
