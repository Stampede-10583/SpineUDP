package frc.robot.bench;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import frc.lib.SpineUDP.SpineUDP.OdometrySample;
import frc.lib.SpineUDP.SpineUDP.SpineClock;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class SyntheticOdometrySourceTest {
  @BeforeAll
  static void initHal() {
    assertTrue(HAL.initialize(500, 0));
  }

  /** Velocities must be the time derivatives of the pose, and theta must stay in (-pi, pi]. */
  @Test
  void velocitiesAreDerivativesOfThePath() {
    SyntheticOdometrySource source = new SyntheticOdometrySource();
    double t0 = SpineClock.now();
    OdometrySample a = new OdometrySample();
    OdometrySample b = new OdometrySample();
    OdometrySample mid = new OdometrySample();
    double h = 1e-5;
    boolean wrapped = false;
    double prevTheta = Double.NaN;
    for (double t = t0; t < t0 + 60; t += 0.05) {
      source.fillAt(t - h, a);
      source.fillAt(t + h, b);
      source.fillAt(t, mid);
      assertEquals(t, mid.timestamp);
      assertTrue(mid.theta > -Math.PI && mid.theta <= Math.PI, "theta out of range: " + mid.theta);
      assertEquals((b.x - a.x) / (2 * h), mid.vx, 1e-6, "vx at " + t);
      assertEquals((b.y - a.y) / (2 * h), mid.vy, 1e-6, "vy at " + t);
      double dTheta = Math.atan2(Math.sin(b.theta - a.theta), Math.cos(b.theta - a.theta));
      assertEquals(dTheta / (2 * h), mid.omega, 1e-6, "omega at " + t);
      // One 50 ms step turns well under pi, so a jump of more than pi is the +/-pi wrap.
      wrapped |= Math.abs(mid.theta - prevTheta) > Math.PI;
      prevTheta = mid.theta;
    }
    assertTrue(wrapped, "heading should wrap through +/-pi within a minute");
  }

  @Test
  void startsAtTheOrigin() {
    SyntheticOdometrySource source = new SyntheticOdometrySource();
    OdometrySample s = new OdometrySample();
    assertTrue(source.fill(s));
    // Only the few ms between construction and fill() separate the two; allow a slow test JVM.
    assertEquals(0.0, s.x, 0.05);
    assertEquals(0.0, s.y, 0.05);
  }
}
