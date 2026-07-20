package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * Distance-to-setpoint lookup for the shooter: maps the horizontal distance to the goal (meters)
 * onto a flywheel speed (rotor rps) and a hood position (mechanism rotations, within
 * {@code [HOOD_MIN_ROTATIONS, HOOD_MAX_ROTATIONS]}). Values between table points are linearly
 * interpolated; outside the table they linearly EXTRAPOLATE along the slope of the two nearest
 * points instead of clamping, so a distance beyond calibration degrades gracefully instead of
 * sticking to a stale edge value.
 *
 * <p>The flywheel is driven by the closed velocity loop at every distance ({@code ShootCommands}
 * no longer switches to bang-bang for long shots), so there's a single calibration table.
 *
 * <p>The SAME numbers live in {@code tools/shooter_lookup_plot.py}, which renders the
 * speed-vs-distance and hood-vs-distance charts. Keep them in sync when tuning.
 */
public final class ShooterProfile {
  private ShooterProfile() {}

  // {distance, speed, hoodRotations} — placeholder calibration; re-measure on the real robot.
  // hoodRotations must stay within [HOOD_MIN_ROTATIONS=0.0, HOOD_MAX_ROTATIONS=1.7].
  public static final double[][] TABLE = {
    {1.25, 47.5, 0.4},
    {1.75, 50.0, 0.5},
    {2.25, 47.0, 0.6},
    {3.00, 52.0, 0.8},
    {3.75, 55.0, 0.9}
  };

  private static final int COL_SPEED = 1;
  private static final int COL_HOOD = 2;

  private static final InterpolatingDoubleTreeMap SPEED = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap HOOD = new InterpolatingDoubleTreeMap();

  private static final double MIN_DISTANCE_M = TABLE[0][0];
  private static final double MAX_DISTANCE_M = TABLE[TABLE.length - 1][0];

  static {
    for (double[] row : TABLE) {
      SPEED.put(row[0], row[COL_SPEED]);
      HOOD.put(row[0], row[COL_HOOD]);
    }
  }

  /** Flywheel speed (rotor rps) for a given goal distance (meters). */
  public static double speedForDistance(double meters) {
    return lookup(meters, SPEED, COL_SPEED);
  }

  /** Hood position (mechanism rotations) for a given goal distance (meters). */
  public static double hoodRotationsForDistance(double meters) {
    return lookup(meters, HOOD, COL_HOOD);
  }

  private static double lookup(double meters, InterpolatingDoubleTreeMap map, int column) {
    if (meters < MIN_DISTANCE_M) {
      return extrapolate(TABLE[0], TABLE[1], meters, column);
    }
    if (meters > MAX_DISTANCE_M) {
      return extrapolate(TABLE[TABLE.length - 2], TABLE[TABLE.length - 1], meters, column);
    }
    return map.get(meters);
  }

  // Linear extrapolation along the line through (a[0], a[column]) and (b[0], b[column]).
  private static double extrapolate(double[] a, double[] b, double meters, int column) {
    double slope = (b[column] - a[column]) / (b[0] - a[0]);
    return a[column] + slope * (meters - a[0]);
  }
}
