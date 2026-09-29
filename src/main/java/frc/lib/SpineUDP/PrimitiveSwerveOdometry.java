// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.SpineUDP;

/**
 * Swerve wheel+gyro odometry on primitives only, for the Stream 1 odometry thread.
 *
 * <p>Numerically this mirrors WPILib's {@code SwerveDriveOdometry.update}: a chassis twist from the
 * forward-kinematics pseudo-inverse applied to the module displacement vectors, {@code dtheta} taken
 * from the gyro, and the result integrated with the SE(2) exponential map ({@code Pose2d.exp}). It
 * differs only in that it allocates nothing per update &mdash; no {@code Rotation2d}, {@code
 * Twist2d}, {@code SwerveModulePosition} or matrix objects &mdash; and it also produces field-frame
 * chassis velocities from the module velocities.
 *
 * <p>The pseudo-inverse is computed once, in closed form, as {@code (A^T A)^-1 A^T} for the standard
 * swerve inverse-kinematics matrix {@code A} (rows {@code [1, 0, -y]} and {@code [0, 1, x]} per
 * module). That equals WPILib's SVD pseudo-inverse whenever the modules are not all at one point.
 *
 * <p>Not thread-safe by design: exactly one thread owns an instance. No WPILib imports, so it is
 * portable and unit-testable on a desktop JVM.
 */
public final class PrimitiveSwerveOdometry {
  private final int m_numModules;

  /** Forward kinematics, 3 rows by 2N columns, row-major. */
  private final double[] m_forward;

  // Scratch vectors, 2N each.
  private final double[] m_moduleDeltas;
  private final double[] m_moduleVelocities;

  private final double[] m_prevDistances;
  private double m_prevHeading;
  private boolean m_initialized;

  private double m_x;
  private double m_y;
  private double m_cos = 1.0; // pose heading, kept as cos/sin like Rotation2d
  private double m_sin;
  private double m_vx;
  private double m_vy;

  /**
   * @param moduleX module X locations relative to robot center, in meters
   * @param moduleY module Y locations relative to robot center, in meters, same order as {@code
   *     moduleX}
   */
  public PrimitiveSwerveOdometry(double[] moduleX, double[] moduleY) {
    if (moduleX.length != moduleY.length || moduleX.length < 2) {
      throw new IllegalArgumentException("need at least two modules with matching x/y arrays");
    }
    m_numModules = moduleX.length;
    m_forward = pseudoInverse(moduleX, moduleY);
    m_moduleDeltas = new double[2 * m_numModules];
    m_moduleVelocities = new double[2 * m_numModules];
    m_prevDistances = new double[m_numModules];
  }

  /**
   * Returns the closed-form pseudo-inverse {@code (A^T A)^-1 A^T} of the swerve inverse-kinematics
   * matrix, as 3 rows by 2N columns, row-major.
   */
  private static double[] pseudoInverse(double[] mx, double[] my) {
    int n = mx.length;
    double sx = 0;
    double sy = 0;
    double sr2 = 0;
    for (int i = 0; i < n; i++) {
      sx += mx[i];
      sy += my[i];
      sr2 += mx[i] * mx[i] + my[i] * my[i];
    }
    // A^T A = [[n, 0, -sy], [0, n, sx], [-sy, sx, sr2]]
    double[][] m = {{n, 0, -sy}, {0, n, sx}, {-sy, sx, sr2}};
    double[][] inv = invert3x3(m);

    double[] forward = new double[3 * 2 * n];
    for (int i = 0; i < n; i++) {
      // A^T column 2i is [1, 0, -y_i], column 2i+1 is [0, 1, x_i].
      for (int r = 0; r < 3; r++) {
        forward[r * 2 * n + 2 * i] = inv[r][0] - inv[r][2] * my[i];
        forward[r * 2 * n + 2 * i + 1] = inv[r][1] + inv[r][2] * mx[i];
      }
    }
    return forward;
  }

  private static double[][] invert3x3(double[][] a) {
    double c00 = a[1][1] * a[2][2] - a[1][2] * a[2][1];
    double c01 = a[1][2] * a[2][0] - a[1][0] * a[2][2];
    double c02 = a[1][0] * a[2][1] - a[1][1] * a[2][0];
    double det = a[0][0] * c00 + a[0][1] * c01 + a[0][2] * c02;
    if (Math.abs(det) < 1e-12) {
      throw new IllegalArgumentException("module locations are degenerate");
    }
    double inv = 1.0 / det;
    return new double[][] {
      {c00 * inv, (a[0][2] * a[2][1] - a[0][1] * a[2][2]) * inv, (a[0][1] * a[1][2] - a[0][2] * a[1][1]) * inv},
      {c01 * inv, (a[0][0] * a[2][2] - a[0][2] * a[2][0]) * inv, (a[0][2] * a[1][0] - a[0][0] * a[1][2]) * inv},
      {c02 * inv, (a[0][1] * a[2][0] - a[0][0] * a[2][1]) * inv, (a[0][0] * a[1][1] - a[0][1] * a[1][0]) * inv}
    };
  }

  /**
   * Resets the pose. The next {@link #update} measures motion relative to the given wheel distances
   * and heading.
   *
   * @param x X position, in meters
   * @param y Y position, in meters
   * @param headingRadians gyro heading at this pose, in radians
   * @param distancesMeters current module drive distances, in meters
   */
  public void reset(double x, double y, double headingRadians, double[] distancesMeters) {
    m_x = x;
    m_y = y;
    m_cos = Math.cos(headingRadians);
    m_sin = Math.sin(headingRadians);
    m_prevHeading = headingRadians;
    System.arraycopy(distancesMeters, 0, m_prevDistances, 0, m_numModules);
    m_vx = 0;
    m_vy = 0;
    m_initialized = true;
  }

  /**
   * Integrates one sample. The first call after construction resets to the origin at the given
   * heading instead of integrating.
   *
   * @param headingRadians gyro heading, CCW positive, in radians (continuous or wrapped)
   * @param distancesMeters module drive distances, in meters
   * @param anglesRadians module steer angles, in radians
   * @param velocitiesMps module drive velocities, in meters per second
   */
  public void update(
      double headingRadians,
      double[] distancesMeters,
      double[] anglesRadians,
      double[] velocitiesMps) {
    if (!m_initialized) {
      reset(0.0, 0.0, headingRadians, distancesMeters);
    }

    for (int i = 0; i < m_numModules; i++) {
      double c = Math.cos(anglesRadians[i]);
      double s = Math.sin(anglesRadians[i]);
      double d = distancesMeters[i] - m_prevDistances[i];
      m_moduleDeltas[2 * i] = d * c;
      m_moduleDeltas[2 * i + 1] = d * s;
      m_moduleVelocities[2 * i] = velocitiesMps[i] * c;
      m_moduleVelocities[2 * i + 1] = velocitiesMps[i] * s;
      m_prevDistances[i] = distancesMeters[i];
    }

    int cols = 2 * m_numModules;
    double dx = 0;
    double dy = 0;
    double vxRobot = 0;
    double vyRobot = 0;
    for (int j = 0; j < cols; j++) {
      dx += m_forward[j] * m_moduleDeltas[j];
      dy += m_forward[cols + j] * m_moduleDeltas[j];
      vxRobot += m_forward[j] * m_moduleVelocities[j];
      vyRobot += m_forward[cols + j] * m_moduleVelocities[j];
    }

    // dtheta = angle.minus(previousAngle), wrapped to (-pi, pi] as Rotation2d does.
    double newCos = Math.cos(headingRadians);
    double newSin = Math.sin(headingRadians);
    double dtheta =
        Math.atan2(newSin * m_cos - newCos * m_sin, newCos * m_cos + newSin * m_sin);
    m_prevHeading = headingRadians;

    // Pose2d.exp: twist in the previous pose's frame.
    double sinT = Math.sin(dtheta);
    double cosT = Math.cos(dtheta);
    double s;
    double c;
    if (Math.abs(dtheta) < 1e-9) {
      s = 1.0 - 1.0 / 6.0 * dtheta * dtheta;
      c = 0.5 * dtheta;
    } else {
      s = sinT / dtheta;
      c = (1 - cosT) / dtheta;
    }
    double tx = dx * s - dy * c;
    double ty = dx * c + dy * s;
    m_x += tx * m_cos - ty * m_sin;
    m_y += tx * m_sin + ty * m_cos;

    // The resulting pose takes the gyro heading, as WPILib's odometry does.
    m_cos = newCos;
    m_sin = newSin;

    // Robot-relative chassis velocity rotated into the odometry frame.
    m_vx = vxRobot * newCos - vyRobot * newSin;
    m_vy = vxRobot * newSin + vyRobot * newCos;
  }

  /**
   * Returns the X position, in meters.
   *
   * @return the X position
   */
  public double getX() {
    return m_x;
  }

  /**
   * Returns the Y position, in meters.
   *
   * @return the Y position
   */
  public double getY() {
    return m_y;
  }

  /**
   * Returns the heading wrapped to (-pi, pi], in radians.
   *
   * @return the heading
   */
  public double getTheta() {
    return Math.atan2(m_sin, m_cos);
  }

  /**
   * Returns the field-frame X velocity from the most recent update, in meters per second.
   *
   * @return the X velocity
   */
  public double getVx() {
    return m_vx;
  }

  /**
   * Returns the field-frame Y velocity from the most recent update, in meters per second.
   *
   * @return the Y velocity
   */
  public double getVy() {
    return m_vy;
  }

  /**
   * Returns the gyro heading passed to the most recent update, unwrapped, in radians.
   *
   * @return the last heading
   */
  public double getLastHeading() {
    return m_prevHeading;
  }
}
