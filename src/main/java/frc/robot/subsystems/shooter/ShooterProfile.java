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
 * <p>The calibration is split into two tables at the flywheel control-mode switchover distance
 * ({@code ShootCommands} drives the flywheel with the closed velocity loop below it, bang-bang
 * at/above it — see {@code SHOOTER_CONTROL_MODE_DISTANCE_M}), so each regime can be re-tuned on
 * the field independently. The boundary distance is duplicated as the last row of {@link
 * #PID_TABLE} and the first row of {@link #BANGBANG_TABLE} so both curves agree exactly at the
 * switchover.
 *
 * <p>The SAME numbers live in {@code tools/shooter_lookup_plot.py}, which renders the
 * speed-vs-distance and hood-vs-distance charts. Keep the two tables in sync when tuning.
 */
public final class ShooterProfile {
  private ShooterProfile() {}

  // {distance, speed, hoodRotations} — placeholder calibration; re-measure on the real robot.
  // hoodRotations must stay within [HOOD_MIN_ROTATIONS=0.0, HOOD_MAX_ROTATIONS=1.7].
  public static final double[][] PID_TABLE = {
    {1.25, 47.5, 0.25},
    {1.75, 50, 0.4},
    {2.25, 50.0, 0.53},
  };
  public static final double[][] BANGBANG_TABLE = {
    {2.25, 40, 0.6},
    {2.50, 42, 0.6},
    {3.00, 47.5, 0.7},
    {3.50, 51.0, 0.8},
  };

  private static final int COL_SPEED = 1;
  private static final int COL_HOOD = 2;

  private static final InterpolatingDoubleTreeMap PID_SPEED = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap PID_HOOD = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap BANGBANG_SPEED = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap BANGBANG_HOOD = new InterpolatingDoubleTreeMap();

  // The two tables meet at this distance, so it's both "the max of PID_TABLE" and "the min of
  // BANGBANG_TABLE" — used to pick which table a given distance falls into.
  private static final double SWITCHOVER_DISTANCE_M = PID_TABLE[PID_TABLE.length - 1][0];
  private static final double MIN_DISTANCE_M = PID_TABLE[0][0];
  private static final double MAX_DISTANCE_M = BANGBANG_TABLE[BANGBANG_TABLE.length - 1][0];

  static {
    for (double[] row : PID_TABLE) {
      PID_SPEED.put(row[0], row[COL_SPEED]);
      PID_HOOD.put(row[0], row[COL_HOOD]);
    }
    for (double[] row : BANGBANG_TABLE) {
      BANGBANG_SPEED.put(row[0], row[COL_SPEED]);
      BANGBANG_HOOD.put(row[0], row[COL_HOOD]);
    }
  }

  /** Flywheel speed (rotor rps) for a given goal distance (meters). */
  public static double speedForDistance(double meters) {
    return lookup(meters, PID_SPEED, BANGBANG_SPEED, COL_SPEED);
  }

  /** Hood position (mechanism rotations) for a given goal distance (meters). */
  public static double hoodRotationsForDistance(double meters) {
    return lookup(meters, PID_HOOD, BANGBANG_HOOD, COL_HOOD);
  }

  private static double lookup(
      double meters,
      InterpolatingDoubleTreeMap pidMap,
      InterpolatingDoubleTreeMap bangBangMap,
      int column) {
    if (meters < MIN_DISTANCE_M) {
      return extrapolate(PID_TABLE[0], PID_TABLE[1], meters, column);
    }
    if (meters > MAX_DISTANCE_M) {
      return extrapolate(
          BANGBANG_TABLE[BANGBANG_TABLE.length - 2],
          BANGBANG_TABLE[BANGBANG_TABLE.length - 1],
          meters,
          column);
    }
    return meters <= SWITCHOVER_DISTANCE_M ? pidMap.get(meters) : bangBangMap.get(meters);
  }

  // Linear extrapolation along the line through (a[0], a[column]) and (b[0], b[column]).
  private static double extrapolate(double[] a, double[] b, double meters, int column) {
    double slope = (b[column] - a[column]) / (b[0] - a[0]);
    return a[column] + slope * (meters - a[0]);
  }
}
