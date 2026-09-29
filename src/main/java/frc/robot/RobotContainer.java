// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.lib.SpineUDP.SpineUDP;
import frc.lib.SpineUDP.SpineUDP.OdometrySender;
import frc.lib.SpineUDP.SpineUDP.TriggerTableSender;
import frc.lib.SpineUDP.SpineUDP.VisionPoseReceiver;
import frc.robot.Constants.VisionLinkConstants;
import frc.robot.bench.BenchTriggerPulser;
import frc.robot.bench.SpineBenchSubsystem;
import frc.robot.bench.SyntheticOdometrySource;

/**
 * Wires the bench RIO: the Spine link driven by synthetic producers, with no drivetrain.
 *
 * <p>Streams 1 and 3 go to the host from {@link VisionLinkConstants#orinHost()}; Stream 2 is
 * received on all interfaces and monitored by {@link SpineBenchSubsystem}.
 */
public class RobotContainer {
  private VisionPoseReceiver m_poseReceiver;
  private TriggerTableSender m_triggerSender;
  private BenchTriggerPulser m_pulser;
  private OdometrySender m_odometrySender;
  private SyntheticOdometrySource m_odometrySource;
  private SpineBenchSubsystem m_bench;

  /** Builds and starts the bench. */
  public RobotContainer() {
    startVisionLink();
  }

  /**
   * Creates the link and starts it, consumers before producers: the Stream 2 receiver, the Stream
   * 3 sender, the trigger pulser that feeds it, then the Stream 1 sender.
   */
  private void startVisionLink() {
    String host = VisionLinkConstants.orinHost();
    SpineUDP.Config config = VisionLinkConstants.kLinkConfig(host);
    System.out.println("[SpineBench] Orin host " + host);

    m_poseReceiver = new VisionPoseReceiver(config);
    m_triggerSender = new TriggerTableSender(config);
    m_pulser =
        new BenchTriggerPulser(
            m_triggerSender,
            VisionLinkConstants.kTriggerRateHz,
            VisionLinkConstants.kTriggerDioChannel,
            VisionLinkConstants.kTriggerPulseSeconds);
    m_odometrySender = new OdometrySender(config);
    m_odometrySource = new SyntheticOdometrySource();
    m_bench =
        new SpineBenchSubsystem(
            m_poseReceiver, m_odometrySource, VisionLinkConstants.kBenchTable);

    m_poseReceiver.start();
    m_triggerSender.start();
    m_pulser.start();
    m_odometrySender.start(m_odometrySource);
  }
}
