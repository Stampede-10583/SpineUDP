// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.SpineUDP.SpineUDP;

/** Robot-wide constants for the Spine bench RIO. */
public final class Constants {

  /** Spine vision link: addresses, ports, rates and the bench trigger pulser. */
  public static final class VisionLinkConstants {
    /** The Orin's static address on the robot network. Placeholder until the Orin is set up. */
    public static final String kOrinIp = "10.105.83.11";

    /** Where the Orin end runs in simulation: this machine. */
    public static final String kSimOrinIp = "127.0.0.1";

    /**
     * Preferences key that overrides the Orin host at startup, so a real RIO can be pointed at a
     * PC acting as the coprocessor without a redeploy. Restart robot code after changing it.
     */
    public static final String kOrinHostPrefKey = "Spine/orinHost";

    public static final int kOdometryPort = 5801;
    public static final int kVisionPosePort = 5802;
    public static final int kTriggerTablePort = 5803;

    public static final int kOdometryRateHz = 200;
    public static final int kVisionRingSize = 64;
    public static final int kVisionSocketRcvBufBytes = 1 << 16;
    public static final double kVisionMaxAgeSeconds = 1.5;
    public static final double kStatsPublishPeriodSeconds = 1.0;
    public static final String kStatsTable = "VisionLink";

    /** Camera trigger rate; one Stream 3 packet per trigger. */
    public static final double kTriggerRateHz = 60.0;

    /** DIO channel wired to the camera trigger inputs, or -1 to run without a DIO pulse. */
    public static final int kTriggerDioChannel = -1;

    /** Width of the hardware trigger pulse. Unchecked against the cameras' minimum. */
    public static final double kTriggerPulseSeconds = 100e-6;

    /** NetworkTables table for the bench's own telemetry (vision pose, odometry, ordering). */
    public static final String kBenchTable = "SpineBench";

    /**
     * Returns the Orin host: the {@value #kOrinHostPrefKey} Preference if set, otherwise {@link
     * #kOrinIp} on a real RIO or {@link #kSimOrinIp} in simulation. Creates the Preference with the
     * default so it shows up in NetworkTables for editing.
     *
     * @return the host Streams 1 and 3 are sent to
     */
    public static String orinHost() {
      String fallback = RobotBase.isSimulation() ? kSimOrinIp : kOrinIp;
      Preferences.initString(kOrinHostPrefKey, fallback);
      String host = Preferences.getString(kOrinHostPrefKey, fallback).trim();
      return host.isEmpty() ? fallback : host;
    }

    /**
     * Builds the link configuration for the given Orin host.
     *
     * @param host the host Streams 1 and 3 are sent to
     * @return the link configuration
     */
    public static SpineUDP.Config kLinkConfig(String host) {
      return new SpineUDP.Config(
          host,
          kOdometryPort,
          kVisionPosePort,
          kTriggerTablePort,
          kOdometryRateHz,
          kVisionRingSize,
          kVisionSocketRcvBufBytes,
          kVisionMaxAgeSeconds,
          kStatsPublishPeriodSeconds,
          kStatsTable);
    }

    private VisionLinkConstants() {}
  }

  private Constants() {}
}
