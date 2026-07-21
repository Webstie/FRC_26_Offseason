// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.vision.VisionConstants;

/**
 * Field-relative target locations, with red/blue alliance (红蓝方) handling.
 *
 * <p>Targets are stored in the BLUE-alliance frame (WPILib field origin at the blue-alliance corner,
 * +x toward the red wall, +y left). {@link #goal()} returns the target for the current alliance,
 * mirroring blue coordinates onto the red side at runtime so the same value works for both.
 */
public final class FieldConstants {
  private FieldConstants() {}

  // Field dimensions read from the loaded 2026 AprilTag layout, so the mirror auto-matches the field.
  public static final double FIELD_LENGTH = VisionConstants.APRIL_TAG_LAYOUT.getFieldLength();
  public static final double FIELD_WIDTH = VisionConstants.APRIL_TAG_LAYOUT.getFieldWidth();

  // Blue-alliance goal ("炮塔") in field coordinates (meters), from the 25-26-swerve hub center.
  // y ≈ FIELD_WIDTH/2 (field centerline), so the red mirror is identical under point or mirror symmetry.
  // Verify this is the correct 2026 goal location before competition.
  public static final Translation2d BLUE_GOAL = new Translation2d(4.625, 4.035);

  // Feed corners near the blue alliance wall/loading zone (ported from 25-26-swerve's
  // BLUE_CORNER_LEFT/RIGHT, rescaled to this field's actual width) — the "feed" shot (see
  // ShootCommands.feed) lobs to whichever is closer instead of aiming at the goal. Placeholder
  // positions; verify against the real 2026 field.
  public static final Translation2d BLUE_FEED_CORNER_LEFT = new Translation2d(1.5, FIELD_WIDTH - 1.0);
  public static final Translation2d BLUE_FEED_CORNER_RIGHT = new Translation2d(1.5, 1.0);

  /** True if the FMS/Driver Station reports the red alliance (defaults to blue when unknown). */
  public static boolean isRedAlliance() {
    return DriverStation.getAlliance().map(a -> a == Alliance.Red).orElse(false);
  }

  /**
   * Mirrors a blue-frame point across the field center — point symmetry {@code (FIELD_LENGTH - x,
   * FIELD_WIDTH - y)}, matching a rotationally-symmetric field. If the 2026 field turns out to be
   * mirror-symmetric instead, drop the Y flip (use {@code FIELD_WIDTH - y} -> {@code y}) here.
   */
  private static Translation2d mirror(Translation2d blue) {
    return new Translation2d(FIELD_LENGTH - blue.getX(), FIELD_WIDTH - blue.getY());
  }

  /** The goal location for the current alliance. Blue (or unknown) returns {@link #BLUE_GOAL}. */
  public static Translation2d goal() {
    return isRedAlliance() ? mirror(BLUE_GOAL) : BLUE_GOAL;
  }

  /** Horizontal distance (meters) from a robot translation to the current alliance goal. */
  public static double distanceToGoal(Translation2d robot) {
    return robot.getDistance(goal());
  }

  /**
   * Whichever feed corner (left or right) is closer to {@code robot}, mirrored for the current
   * alliance. Used by the "feed" shot instead of always picking one side, so it works whichever
   * half of the field the robot ends up on.
   */
  public static Translation2d nearestFeedCorner(Translation2d robot) {
    Translation2d left = isRedAlliance() ? mirror(BLUE_FEED_CORNER_LEFT) : BLUE_FEED_CORNER_LEFT;
    Translation2d right = isRedAlliance() ? mirror(BLUE_FEED_CORNER_RIGHT) : BLUE_FEED_CORNER_RIGHT;
    return robot.getDistance(left) <= robot.getDistance(right) ? left : right;
  }

  /** Horizontal distance (meters) from a robot translation to the nearest feed corner. */
  public static double distanceToFeedCorner(Translation2d robot) {
    return robot.getDistance(nearestFeedCorner(robot));
  }

  /**
   * Adds the alliance-based "operator perspective" offset to a TRUE absolute (blue-frame) heading:
   * blue sees downfield as 0 rad, red as 180 rad (mirrors CTRE's {@code
   * setOperatorPerspectiveForward}). Use this ONLY for interpreting what the driver's stick
   * considers "forward" in field-relative driving -- e.g. as the reference angle passed to {@code
   * ChassisSpeeds.fromFieldRelativeSpeeds}. NEVER use it for the pose estimator's actual rotation
   * (that must stay the true blue-frame value for auto-aim / PathPlanner to work), and never for
   * chassis-rotation targets computed from field geometry (e.g. the auto-aim heading PID) --
   * those already come out correctly mirrored via {@link #goal()} / {@link #nearestFeedCorner}.
   *
   * <p>Every place that drives the chassis field-relative off a live joystick MUST go through this
   * (or hand-roll the same offset) -- missing one is exactly how a driver ends up with mirrored
   * controls on red for that one action while everything else is fine. Known call sites: {@code
   * DriveCommands.joystickDrive} (default teleop drive) and {@code ShootCommands.autoShoot}/{@code
   * feed} (translation while aiming/shooting).
   */
  public static Rotation2d operatorForward(Rotation2d trueHeading) {
    return trueHeading.plus(isRedAlliance() ? Rotation2d.k180deg : Rotation2d.kZero);
  }
}
