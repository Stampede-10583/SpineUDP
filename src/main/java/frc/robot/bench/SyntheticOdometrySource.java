// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.bench;

import frc.lib.SpineUDP.SpineUDP.OdometrySample;
import frc.lib.SpineUDP.SpineUDP.OdometrySource;
import frc.lib.SpineUDP.SpineUDP.SpineClock;

/**
 * Stream 1 source for the bench RIO: a closed-form smooth path instead of real swerve sensors.
 *
 * <p>The robot drives a figure-eight (a 1:2 Lissajous curve) while spinning steadily with a
 * wobble, so the heading wraps through +/-pi and every field of the packet changes. Position,
 * heading and velocities are all derived from the same functions of time, so they agree with each
 * other exactly: {@code vx = dx/dt}, {@code vy = dy/dt}, {@code omega = dtheta/dt}.
 *
 * <p>Like a real sampler's odometry frame, the path starts at {@code x = 0, y = 0} with a nonzero
 * heading at construction time, and is never reset. It needs no hardware, allocates nothing, and
 * timestamps come from {@link SpineClock#now()}.
 *
 * <p>{@link #fillAt} is a pure function of its time argument and final fields, so the main thread
 * may call it for telemetry while the odometry thread calls {@link #fill}.
 */
public final class SyntheticOdometrySource implements OdometrySource {
  private static final double kAmpX = 2.0; // m
  private static final double kAmpY = 1.0; // m
  private static final double kFreqX = 0.4; // rad/s
  private static final double kFreqY = 0.8; // rad/s
  private static final double kHeading0 = 0.5; // rad, "gyro heading" at start
  private static final double kSpin = 0.5; // rad/s, steady rotation
  private static final double kWobbleAmp = 0.8; // rad
  private static final double kWobbleFreq = 0.7; // rad/s

  private final double m_startTime;

  /** Creates a source whose path starts at the origin now. */
  public SyntheticOdometrySource() {
    m_startTime = SpineClock.now();
  }

  @Override
  public boolean fill(OdometrySample dest) {
    fillAt(SpineClock.now(), dest);
    return true;
  }

  /**
   * Fills {@code dest} with the path's state at an explicit controller time. Thread-safe.
   *
   * @param timestamp controller timestamp, in seconds
   * @param dest sample to overwrite
   */
  public void fillAt(double timestamp, OdometrySample dest) {
    double t = timestamp - m_startTime;
    double x = kAmpX * Math.sin(kFreqX * t);
    double y = kAmpY * Math.sin(kFreqY * t);
    double rawTheta = kHeading0 + kSpin * t + kWobbleAmp * Math.sin(kWobbleFreq * t);
    // atan2 wraps to (-pi, pi], the range Stream 1 specifies, without allocating.
    double theta = Math.atan2(Math.sin(rawTheta), Math.cos(rawTheta));
    double vx = kAmpX * kFreqX * Math.cos(kFreqX * t);
    double vy = kAmpY * kFreqY * Math.cos(kFreqY * t);
    double omega = kSpin + kWobbleAmp * kWobbleFreq * Math.cos(kWobbleFreq * t);
    dest.set(timestamp, x, y, theta, vx, vy, omega);
  }
}
