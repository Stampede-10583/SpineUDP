"""A fake roboRIO for tests: sends Streams 1 and 3, records every Stream 2 datagram.

It keeps its own "controller clock" (a monotonic offset), the way the real RIO keeps
MathSharedStore time, and builds packets exactly as SpineUDP.java does. The trigger stream can
inject the network misbehaviour the companion must handle: coalesced packets, duplicates, a
reordered older packet, a burst of lost packets, a junk datagram, and a robot-program restart.
"""

import socket
import struct
import threading
import time


class FakeRio:

    def __init__(self, orin_host, odometry_port, trigger_port, pose_port, clock_base=500.0):
        self._orin_odom = (orin_host, odometry_port)
        self._orin_trig = (orin_host, trigger_port)
        self._tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx.bind(('127.0.0.1', pose_port))
        self._rx.settimeout(0.2)
        self._t0 = time.monotonic()
        self._clock_base = clock_base

        self.poses = []                 # (datagram bytes) in arrival order
        self.trigger_ts_bytes = set()   # raw 8-byte timestamps of every trigger sent
        self.junk_sent = 0
        self.lost_triggers = 0          # triggers that never appeared in any delivered packet
        self._stop = threading.Event()
        self._rx_thread = threading.Thread(target=self._receive, daemon=True)

    # ---------------------------------------------------------------------------------------
    def now(self):
        """The fake controller clock: never resets, like the RIO's across a program restart."""
        return self._clock_base + (time.monotonic() - self._t0)

    def _receive(self):
        while not self._stop.is_set():
            try:
                data, _ = self._rx.recvfrom(256)
            except socket.timeout:
                continue
            self.poses.append(data)

    # ---------------------------------------------------------------------------------------
    @staticmethod
    def trigger_packet(packet_seq, table):
        """table: list of (triggerSeq, timestamp), most recent first, up to 5 entries."""
        out = struct.pack('<q', packet_seq)
        for i in range(5):
            if i < len(table):
                out += struct.pack('<qd', *table[i])
            else:
                out += struct.pack('<qd', -1, 0.0)
        return out

    def run(self, seconds, trigger_hz=60.0, odom_hz=200.0, faults=True):
        """Runs one robot-program run: seq numbers from 0, clock continuing."""
        self._rx_thread.is_alive() or self._rx_thread.start()
        odom_seq = 0
        trig_seq = 0
        packet_seq = 0
        table = []          # (seq, ts), most recent first
        history = []        # every packet built, for the reordered-packet fault
        end = time.monotonic() + seconds
        next_odom = time.monotonic()
        next_trig = time.monotonic()
        while time.monotonic() < end:
            now_wall = time.monotonic()
            if now_wall >= next_odom:
                self._tx.sendto(struct.pack('<q7d', odom_seq, self.now(), 0.01 * odom_seq, 0.0,
                                            0.0, 0.5, 0.0, 0.0), self._orin_odom)
                odom_seq += 1
                next_odom += 1.0 / odom_hz
            if now_wall >= next_trig:
                ts = self.now()
                self.trigger_ts_bytes.add(struct.pack('<d', ts))
                table = [(trig_seq, ts)] + table[:4]
                pkt = self.trigger_packet(packet_seq, table)
                history.append(pkt)
                packet_seq += 1
                k = trig_seq
                trig_seq += 1
                next_trig += 1.0 / trigger_hz

                if faults and 100 <= k <= 105:
                    continue  # 6 packets lost in a row: triggers 100 and 101 are never seen
                if faults and k % 25 == 7:
                    continue  # coalesced: the next packet carries this trigger too
                self._tx.sendto(pkt, self._orin_trig)
                if faults and k % 40 == 13:
                    self._tx.sendto(pkt, self._orin_trig)            # duplicate
                if faults and k == 50:
                    self._tx.sendto(history[-4], self._orin_trig)    # older packet, reordered
                if faults and k == 60:
                    self._tx.sendto(b'\x00' * 10, self._orin_trig)   # junk
                    self.junk_sent += 1
            time.sleep(0.0005)
        if faults and trig_seq > 106:
            self.lost_triggers += 2

    def close(self):
        self._stop.set()
        if self._rx_thread.is_alive():
            self._rx_thread.join(timeout=1.0)
        self._tx.close()
        self._rx.close()
