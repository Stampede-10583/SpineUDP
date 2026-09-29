// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.bench;

import edu.wpi.first.wpilibj.DigitalOutput;
import frc.lib.SpineUDP.SpineUDP.HalPeriodicThread;
import frc.lib.SpineUDP.SpineUDP.SpineClock;
import frc.lib.SpineUDP.SpineUDP.TriggerTableSender;

/**
 * Stream 3 driver for the bench RIO: fires a camera trigger at a fixed rate.
 *
 * <p>Each tick optionally starts a hardware pulse on a DIO (non-blocking; the FPGA times the pulse
 * width), then records the trigger at the current controller time. This thread is the <b>only</b>
 * caller of {@link TriggerTableSender#recordTrigger(double)}, as the trigger table's single-writer
 * seqlock requires.
 *
 * <p>With the DIO disabled (channel -1) it needs no hardware, so it runs on a bare RIO and in
 * simulation. Allocation-free per tick.
 */
public final class BenchTriggerPulser {
  private final TriggerTableSender m_sender;
  private final DigitalOutput m_dio;
  private final double m_pulseSeconds;
  private final HalPeriodicThread m_thread;

  /**
   * @param sender the Stream 3 sender to record triggers into
   * @param rateHz trigger rate
   * @param dioChannel DIO channel to pulse, or -1 for none
   * @param pulseSeconds hardware pulse width
   */
  public BenchTriggerPulser(
      TriggerTableSender sender, double rateHz, int dioChannel, double pulseSeconds) {
    m_sender = sender;
    m_dio = dioChannel >= 0 ? new DigitalOutput(dioChannel) : null;
    m_pulseSeconds = pulseSeconds;
    m_thread = new HalPeriodicThread("VisionLink-TriggerPulser", 1.0 / rateHz, this::tick, 0);
  }

  private void tick() {
    if (m_dio != null) {
      m_dio.pulse(m_pulseSeconds);
    }
    // Read the clock after the pulse call returns: the FPGA starts the pulse at the end of that
    // call, so this is the closer of the two possible reads to the rising edge.
    m_sender.recordTrigger(SpineClock.now());
  }

  /** Starts pulsing. Idempotent. */
  public void start() {
    m_thread.start();
  }

  /** Stops pulsing. */
  public void stop() {
    m_thread.stop();
  }

  /**
   * Returns the periodic thread, for its late-tick count.
   *
   * @return the periodic thread
   */
  public HalPeriodicThread getPeriodicThread() {
    return m_thread;
  }
}
