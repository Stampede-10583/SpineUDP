// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.bench;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.SpineUDP.SpineUDP.OdometrySample;
import frc.lib.SpineUDP.SpineUDP.SpineClock;
import frc.lib.SpineUDP.SpineUDP.VisionPoseReceiver;
import java.util.function.ObjIntConsumer;

/**
 * The bench RIO's Stream 2 consumer, standing in for the pose estimator of a real robot.
 *
 * <p>Once per robot loop it drains the {@link VisionPoseReceiver} and publishes, under the bench
 * NetworkTables table:
 *
 * <ul>
 *   <li>{@code visionPose}: the latest Stream 2 pose as {@code [x, y, theta]};
 *   <li>{@code visionPoseAgeSec}: controller time now minus that pose's timestamp (-1 before the
 *       first pose);
 *   <li>{@code visionPosesSeen}: Stream 2 poses drained so far;
 *   <li>{@code poseOutOfOrder}: poses whose timestamp was {@code <=} the newest one already seen.
 *       This is the RIO-side end-to-end check of the Orin's ordering guarantee and should stay 0;
 *   <li>{@code odomPose}: the synthetic Stream 1 pose at this instant, as {@code [x, y, theta]},
 *       for side-by-side viewing in AdvantageScope or Glass.
 * </ul>
 *
 * <p>Allocation-free per loop: the drain consumer is held in a field, the arrays are preallocated,
 * and no {@code Pose2d} is built.
 */
public final class SpineBenchSubsystem extends SubsystemBase {
  private final VisionPoseReceiver m_receiver;
  private final SyntheticOdometrySource m_odometry;
  private final ObjIntConsumer<VisionPoseReceiver.PoseView> m_onPose = this::onPose;

  private final DoubleArrayPublisher m_visionPosePub;
  private final DoublePublisher m_visionAgePub;
  private final IntegerPublisher m_posesSeenPub;
  private final IntegerPublisher m_outOfOrderPub;
  private final DoubleArrayPublisher m_odomPosePub;

  private final double[] m_visionPose = new double[3];
  private final double[] m_odomPose = new double[3];
  private final OdometrySample m_odomSample = new OdometrySample();

  private boolean m_haveVision;
  private double m_newestTimestamp;
  private long m_posesSeen;
  private long m_outOfOrder;

  /**
   * @param receiver the Stream 2 receiver to drain; started by the caller
   * @param odometry the synthetic Stream 1 source, evaluated here for telemetry only
   * @param tableName NetworkTables table to publish under
   */
  public SpineBenchSubsystem(
      VisionPoseReceiver receiver, SyntheticOdometrySource odometry, String tableName) {
    m_receiver = receiver;
    m_odometry = odometry;
    NetworkTable table = NetworkTableInstance.getDefault().getTable(tableName);
    m_visionPosePub = table.getDoubleArrayTopic("visionPose").publish();
    m_visionAgePub = table.getDoubleTopic("visionPoseAgeSec").publish();
    m_posesSeenPub = table.getIntegerTopic("visionPosesSeen").publish();
    m_outOfOrderPub = table.getIntegerTopic("poseOutOfOrder").publish();
    m_odomPosePub = table.getDoubleArrayTopic("odomPose").publish();
  }

  @Override
  public void periodic() {
    if (m_receiver.readPackets() > 0) {
      m_receiver.forEachPacket(m_onPose);
    }
    double now = SpineClock.now();

    if (m_haveVision) {
      m_visionPosePub.set(m_visionPose);
      m_visionAgePub.set(now - m_newestTimestamp);
    } else {
      m_visionAgePub.set(-1.0);
    }
    m_posesSeenPub.set(m_posesSeen);
    m_outOfOrderPub.set(m_outOfOrder);

    m_odometry.fillAt(now, m_odomSample);
    m_odomPose[0] = m_odomSample.x;
    m_odomPose[1] = m_odomSample.y;
    m_odomPose[2] = m_odomSample.theta;
    m_odomPosePub.set(m_odomPose);
  }

  private void onPose(VisionPoseReceiver.PoseView pose, int index) {
    double ts = pose.getTimestamp();
    if (m_haveVision && ts <= m_newestTimestamp) {
      m_outOfOrder++;
    } else {
      m_newestTimestamp = ts;
    }
    m_haveVision = true;
    m_posesSeen++;
    m_visionPose[0] = pose.getX();
    m_visionPose[1] = pose.getY();
    m_visionPose[2] = pose.getTheta();
  }

  /**
   * Returns how many Stream 2 poses arrived with a timestamp at or before the newest one seen.
   *
   * @return the out-of-order count
   */
  public long getOutOfOrderCount() {
    return m_outOfOrder;
  }
}
