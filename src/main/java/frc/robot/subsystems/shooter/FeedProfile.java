package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * Distance-to-speed lookup for the FEED shot (lobbed to a feed corner instead of the goal — see
 * {@code ShootCommands.feed}, ported from 25-26-swerve's corner-feed): maps the horizontal distance
 * to the target feed corner (meters) onto a flywheel speed (rotor rps). Same linear interpolation +
 * linear EXTRAPOLATION-past-the-ends behavior as {@link ShooterProfile}, just a single table — a
 * lobbed feed doesn't need the PID/bang-bang control-mode split a hard shot at the goal does.
 *
 * <p>Unlike {@link ShooterProfile}, there's no hood column here: a feed's hood angle is a single
 * fixed value ({@code Constants.FeedConfig.FEED_HOOD_ROTATIONS}), not distance-dependent. FRC2910
 * and FRC6328's 2026 codebases both do this for their cross-field pass shot — pin the hood near max
 * (lob) angle and let speed alone cover the range, rather than re-shaping the trajectory per shot.
 *
 * <p>Table range (3-16m) is picked to bracket where {@code ShootCommands.feed} actually gets used:
 * {@code RobotContainer.pastFeedZone()} switches the right trigger to a feed once the robot is
 * physically past the goal (x≈4.6m from its own wall), and the feed corners sit near that same
 * wall (x≈1.5m) — so the real operating distance starts around 3m right at the switchover and
 * grows from there as the robot continues across the field.
 *
 * <p>The SAME numbers live in {@code tools/feed_lookup_plot.py}, which renders the speed-vs-distance
 * chart. Keep the two in sync when tuning.
 */
public final class FeedProfile {
  private FeedProfile() {}

  // {distance, speed} — placeholder calibration; re-measure on the real robot.
  public static final double[][] TABLE = {
    {3.0, 40.0},
    {5.0, 45.0},
    {7.0, 60.0},
    {9.0, 55.0},
    {12.0, 65.0},
    {16.0, 75.0},
  };

  private static final int COL_SPEED = 1;

  private static final InterpolatingDoubleTreeMap SPEED = new InterpolatingDoubleTreeMap();

  private static final double MIN_DISTANCE_M = TABLE[0][0];
  private static final double MAX_DISTANCE_M = TABLE[TABLE.length - 1][0];

  static {
    for (double[] row : TABLE) {
      SPEED.put(row[0], row[COL_SPEED]);
    }
  }

  /** Flywheel speed (rotor rps) for a given feed-corner distance (meters). */
  public static double speedForDistance(double meters) {
    if (meters < MIN_DISTANCE_M) {
      return extrapolate(TABLE[0], TABLE[1], meters);
    }
    if (meters > MAX_DISTANCE_M) {
      return extrapolate(TABLE[TABLE.length - 2], TABLE[TABLE.length - 1], meters);
    }
    return SPEED.get(meters);
  }

  // Linear extrapolation along the line through (a[0], a[COL_SPEED]) and (b[0], b[COL_SPEED]).
  private static double extrapolate(double[] a, double[] b, double meters) {
    double slope = (b[COL_SPEED] - a[COL_SPEED]) / (b[0] - a[0]);
    return a[COL_SPEED] + slope * (meters - a[0]);
  }
}
