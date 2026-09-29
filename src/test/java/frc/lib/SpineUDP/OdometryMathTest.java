package frc.lib.SpineUDP;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.util.Random;
import org.junit.jupiter.api.Test;

class OdometryMathTest {
  private static final double kHalf = 0.3175; // 12.5 in
  private static final Translation2d[] kLocations = {
    new Translation2d(kHalf, kHalf),
    new Translation2d(kHalf, -kHalf),
    new Translation2d(-kHalf, kHalf),
    new Translation2d(-kHalf, -kHalf)
  };

  /**
   * Drives WPILib's SwerveDriveOdometry and PrimitiveSwerveOdometry with identical noisy module and
   * gyro inputs; the poses and velocities must agree to within floating-point noise.
   */
  @Test
  void primitiveOdometryMatchesWpilib() {
    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(kLocations);
    double[] mx = new double[4];
    double[] my = new double[4];
    for (int i = 0; i < 4; i++) {
      mx[i] = kLocations[i].getX();
      my[i] = kLocations[i].getY();
    }
    PrimitiveSwerveOdometry primitive = new PrimitiveSwerveOdometry(mx, my);

    Random rng = new Random(83);
    double heading = 0.4;
    double[] dist = new double[4];
    double[] ang = new double[4];
    double[] vel = new double[4];
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      positions[i] = new SwerveModulePosition(0, Rotation2d.kZero);
    }

    // The primitive odometry starts at the origin with the gyro's heading; give WPILib the same start.
    SwerveDriveOdometry wpilib =
        new SwerveDriveOdometry(
            kinematics, new Rotation2d(heading), positions, new Pose2d(0, 0, new Rotation2d(heading)));
    primitive.update(heading, dist, ang, vel); // first call resets to the origin at this heading

    double dt = 0.005;
    for (int step = 0; step < 4000; step++) {
      ChassisSpeeds speeds =
          new ChassisSpeeds(
              2.0 * Math.sin(step * 0.003), 1.5 * Math.cos(step * 0.002), 2.5 * Math.sin(step * 0.004));
      SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
      heading += speeds.omegaRadiansPerSecond * dt + rng.nextGaussian() * 1e-4; // gyro != wheels
      for (int i = 0; i < 4; i++) {
        double v = states[i].speedMetersPerSecond * (1 + rng.nextGaussian() * 0.01); // wheel slip
        dist[i] += v * dt;
        ang[i] = states[i].angle.getRadians();
        vel[i] = v;
        positions[i] = new SwerveModulePosition(dist[i], states[i].angle);
      }

      Pose2d expected = wpilib.update(new Rotation2d(heading), positions);
      primitive.update(heading, dist, ang, vel);

      assertEquals(expected.getX(), primitive.getX(), 1e-9, "x at step " + step);
      assertEquals(expected.getY(), primitive.getY(), 1e-9, "y at step " + step);
      assertEquals(
          0.0,
          MathUtil.angleModulus(expected.getRotation().getRadians() - primitive.getTheta()),
          1e-9,
          "theta at step " + step);

      SwerveModuleState[] measured = new SwerveModuleState[4];
      for (int i = 0; i < 4; i++) {
        measured[i] = new SwerveModuleState(vel[i], new Rotation2d(ang[i]));
      }
      ChassisSpeeds field =
          ChassisSpeeds.fromRobotRelativeSpeeds(kinematics.toChassisSpeeds(measured), new Rotation2d(heading));
      assertEquals(field.vxMetersPerSecond, primitive.getVx(), 1e-9, "vx at step " + step);
      assertEquals(field.vyMetersPerSecond, primitive.getVy(), 1e-9, "vy at step " + step);
    }
  }

  /**
   * Zero vision std-devs give a Kalman gain of exactly 1: the estimate snaps to the vision pose. The
   * old Double.MIN_VALUE trick must behave identically.
   */
  @Test
  void zeroStdDevsReplaceTheEstimate() {
    for (double sd : new double[] {0.0, Double.MIN_VALUE}) {
      Estimator e = new Estimator();
      e.driveStraight(0.0, 1.0, 2.0); // odometry-only pose A at (2, 0)
      Pose2d visionPose = new Pose2d(5.0, -1.0, Rotation2d.fromDegrees(30));
      e.estimator.addVisionMeasurement(visionPose, 1.0, VecBuilder.fill(sd, sd, sd));
      assertPoseEquals(visionPose, e.estimator.getEstimatedPosition(), "sd=" + sd);
    }
  }

  /**
   * A pose stamped 100 ms in the past replaces the estimate at that time, and the odometry recorded
   * since then is replayed on top of it.
   */
  @Test
  void pastPoseIsReplayedForward() {
    Estimator e = new Estimator();
    e.driveStraight(0.0, 1.0, 2.0); // 2 m/s forward for 1 s, samples every 5 ms

    Pose2d odomAtVision = e.odometryPoseAt(0.9);
    Pose2d odomNow = e.odometryPoseAt(1.0);
    Pose2d visionPose = new Pose2d(5.0, -1.0, Rotation2d.fromDegrees(30));
    e.estimator.addVisionMeasurement(visionPose, 0.9, VecBuilder.fill(0.0, 0.0, 0.0));

    Pose2d expected = visionPose.plus(odomNow.minus(odomAtVision));
    assertPoseEquals(expected, e.estimator.getEstimatedPosition(), "replayed");
  }

  private static void assertPoseEquals(Pose2d expected, Pose2d actual, String what) {
    assertEquals(expected.getX(), actual.getX(), 1e-9, what + " x");
    assertEquals(expected.getY(), actual.getY(), 1e-9, what + " y");
    assertEquals(
        0.0,
        MathUtil.angleModulus(expected.getRotation().getRadians() - actual.getRotation().getRadians()),
        1e-9,
        what + " theta");
  }

  /** A pose estimator fed a constant-heading straight drive, with a parallel odometry log. */
  private static final class Estimator {
    final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(kLocations);
    final SwerveDrivePoseEstimator estimator;
    final SwerveDriveOdometry odometry;
    final java.util.TreeMap<Double, Pose2d> log = new java.util.TreeMap<>();

    Estimator() {
      estimator =
          new SwerveDrivePoseEstimator(kinematics, Rotation2d.kZero, positions(0.0), Pose2d.kZero);
      odometry = new SwerveDriveOdometry(kinematics, Rotation2d.kZero, positions(0.0));
    }

    void driveStraight(double t0, double t1, double speed) {
      for (int k = 0; ; k++) {
        double t = t0 + k * 0.005;
        if (t > t1 + 1e-9) {
          break;
        }
        SwerveModulePosition[] p = positions(speed * (t - t0));
        estimator.updateWithTime(t, Rotation2d.kZero, p);
        log.put(t, odometry.update(Rotation2d.kZero, p));
      }
    }

    Pose2d odometryPoseAt(double t) {
      return log.floorEntry(t + 1e-9).getValue();
    }

    private static SwerveModulePosition[] positions(double d) {
      SwerveModulePosition[] p = new SwerveModulePosition[4];
      for (int i = 0; i < 4; i++) {
        p[i] = new SwerveModulePosition(d, Rotation2d.kZero);
      }
      return p;
    }
  }
}
