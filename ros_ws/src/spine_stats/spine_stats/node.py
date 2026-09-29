"""spine_stats: Orin-side Spine link telemetry.

- Publishes the companion's counters to NetworkTables under Spine/Orin/* as an NT4 client of the
  RIO, about once per second. These cover only what the Orin knows; the RIO's own counters are
  under VisionLink/*.
- Subscribes read-only to the RIO's VisionLink/* (and the bench's SpineBench/*), so every log
  snapshot records both ends side by side.
- Writes rotating JSON Lines logs: 1 Hz snapshots, every LinkEvent, and NT connects/disconnects.

Parameters:
  nt_server       RIO address for NT4. Empty: find the RIO by team number.
  team            team number used when nt_server is empty (10583)
  nt_identity     NT4 client identity ("spine-orin")
  log_dir         where spine-orin.jsonl is written. Empty: $SPINE_LOG_DIR, else
                  ~/.ros/spine_logs
  log_max_bytes   rotate at this size (10 MB)
  log_backups     rotated files kept (20)
  period_s        snapshot / NT publish period (1.0)
"""

import os
import threading
import time

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from spine_interfaces.msg import AdapterStats, LinkEvent, LinkStats

from spine_stats.jsonl import JsonlLog

try:
    import ntcore
except ImportError:  # pragma: no cover - depends on the environment
    ntcore = None

# LinkStats field -> Spine/Orin topic name (docs/spine-link.md section 6).
ORIN_COUNTERS = {
    'poses_sent': 'posesSent',
    'withheld_no_fix': 'withheldNoFix',
    'withheld_bad_timestamp': 'withheldBadTimestamp',
    'withheld_stale': 'withheldStale',
    'withheld_out_of_order': 'withheldOutOfOrder',
    'send_errors': 'sendErrors',
    'odom_recvd': 'odomRecvd',
    'odom_seq_gaps': 'odomSeqGaps',
    'trig_recvd': 'trigRecvd',
    'triggers_missed': 'triggersMissed',
    'decode_errors': 'decodeErrors',
    'rio_restarts': 'rioRestarts',
}

EVENT_KINDS = {
    LinkEvent.RIO_RESTART: 'RIO_RESTART',
    LinkEvent.TRIGGER_GAP: 'TRIGGER_GAP',
    LinkEvent.DECODE_ERROR: 'DECODE_ERROR',
    LinkEvent.ODOM_GAP: 'ODOM_GAP',
}

RIO_TABLES = ('VisionLink', 'SpineBench')


def stats_to_dict(msg):
    """LinkStats -> {NT topic name: value}, plus the restart timestamp."""
    out = {nt: int(getattr(msg, field)) for field, nt in ORIN_COUNTERS.items()}
    out['lastRestartControllerTimestamp'] = msg.last_restart_controller_timestamp
    return out


class SpineStats(Node):

    def __init__(self):
        super().__init__('spine_stats')
        self.declare_parameter('nt_server', '10.105.83.2')
        self.declare_parameter('team', 10583)
        self.declare_parameter('nt_identity', 'spine-orin')
        self.declare_parameter('log_dir', '')
        self.declare_parameter('log_max_bytes', 10 * 1024 * 1024)
        self.declare_parameter('log_backups', 20)
        self.declare_parameter('period_s', 1.0)

        log_dir = self.get_parameter('log_dir').value or os.environ.get(
            'SPINE_LOG_DIR', os.path.expanduser('~/.ros/spine_logs'))
        self._log = JsonlLog(
            log_dir,
            max_bytes=int(self.get_parameter('log_max_bytes').value),
            backup_count=int(self.get_parameter('log_backups').value))

        self._lock = threading.Lock()
        self._last_stats = None
        self._last_stats_mono = None
        self._frames_unpaired = None
        self._heartbeat = 0

        self.create_subscription(LinkStats, 'spine/stats', self._on_stats, 10)
        self.create_subscription(
            LinkEvent, 'spine/events', self._on_event,
            QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(AdapterStats, 'spine/adapter_stats', self._on_adapter_stats, 10)

        self._nt = None
        self._nt_pubs = {}
        self._start_nt()

        self._log.write('start', log_path=self._log.path, nt=self._nt is not None,
                        nt_server=self.get_parameter('nt_server').value)
        self.get_logger().info(f'logging to {self._log.path} (run {self._log.run_id})')
        self.create_timer(float(self.get_parameter('period_s').value), self._tick)

    # -------------------------------------------------------------------------------------
    # NetworkTables
    # -------------------------------------------------------------------------------------
    def _start_nt(self):
        if ntcore is None:
            self.get_logger().warning(
                'pyntcore not importable: NT publishing disabled, logging only. '
                'Run scripts/setup_host.sh or source scripts/env.sh.')
            return
        inst = ntcore.NetworkTableInstance.create()
        inst.startClient4(self.get_parameter('nt_identity').value)
        server = self.get_parameter('nt_server').value
        if server:
            inst.setServer(server)
        else:
            inst.setServerTeam(int(self.get_parameter('team').value))
        inst.addConnectionListener(True, self._on_nt_connection)
        orin = inst.getTable('Spine/Orin')
        names = list(ORIN_COUNTERS.values()) + ['framesUnpaired', 'statsAgeMs', 'heartbeat']
        self._nt_pubs = {n: orin.getIntegerTopic(n).publish() for n in names}
        # Keep the RIO's tables flowing to this client, read-only.
        self._rio_sub = ntcore.MultiSubscriber(inst, [f'/{t}/' for t in RIO_TABLES])
        self._nt = inst

    def _on_nt_connection(self, event):
        # Runs on an NT thread; JsonlLog is thread-safe.
        info = event.data
        connected = event.is_(ntcore.EventFlags.kConnected)
        self._log.write('nt_connection', connected=connected,
                        remote=f'{info.remote_ip}:{info.remote_port}')

    def _rio_snapshot(self):
        snap = {}
        for name in RIO_TABLES:
            table = self._nt.getTable(name)
            values = {}
            for key in table.getKeys():
                v = table.getEntry(key).getValue()
                if v.isValid():
                    values[key] = v.value()
            snap[name] = values
        return snap

    # -------------------------------------------------------------------------------------
    # ROS callbacks
    # -------------------------------------------------------------------------------------
    def _on_stats(self, msg):
        with self._lock:
            self._last_stats = stats_to_dict(msg)
            self._last_stats_mono = time.monotonic()

    def _on_adapter_stats(self, msg):
        with self._lock:
            self._frames_unpaired = int(msg.frames_unpaired)

    def _on_event(self, msg):
        self._log.write('event', kind=EVENT_KINDS.get(msg.kind, msg.kind),
                        controller_timestamp=msg.controller_timestamp, detail=msg.detail)

    def _tick(self):
        with self._lock:
            stats = dict(self._last_stats) if self._last_stats else None
            age_ms = (None if self._last_stats_mono is None
                      else int((time.monotonic() - self._last_stats_mono) * 1000))
            frames_unpaired = self._frames_unpaired
        self._heartbeat += 1

        if self._nt is not None:
            if stats:
                for name, value in stats.items():
                    if name in self._nt_pubs:
                        self._nt_pubs[name].set(value)
            if frames_unpaired is not None:
                self._nt_pubs['framesUnpaired'].set(frames_unpaired)
            self._nt_pubs['statsAgeMs'].set(age_ms if age_ms is not None else -1)
            self._nt_pubs['heartbeat'].set(self._heartbeat)

        record = {'orin': stats, 'statsAgeMs': age_ms, 'framesUnpaired': frames_unpaired}
        if self._nt is not None:
            record['ntConnected'] = self._nt.isConnected()
            record['rio'] = self._rio_snapshot()
        self._log.write('snapshot', **record)

    def destroy_node(self):
        self._log.write('stop')
        self._log.close()
        if self._nt is not None:
            ntcore.NetworkTableInstance.destroy(self._nt)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SpineStats()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
