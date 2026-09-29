// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The Spine bench RIO. The link runs on its own threads in every mode, enabled or not; the main
 * loop only runs the scheduler, which drives {@link frc.robot.bench.SpineBenchSubsystem}.
 */
public class Robot extends TimedRobot {
  @SuppressWarnings("unused") // holds the link and its threads for the life of the program
  private final RobotContainer m_robotContainer;

  /** Builds the bench and starts the link. */
  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }
}
