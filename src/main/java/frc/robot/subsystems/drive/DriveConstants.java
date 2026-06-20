package frc.robot.subsystems.drive;

/**
 * Tunables for the (currently sim-only) MK4n swerve drivetrain.
 *
 * <p>The chassis dimensions/mass are estimates — replace them with your real robot's numbers when
 * you have hardware. The MK4n gear ratios, wheel diameter and steer inertia are NOT here: they live
 * inside maple-sim's {@code COTS.ofMark4n(...)} preset (drive L1+=7.13 / L2=5.9 / L3+=5.36,
 * steer 18.75:1, 4" wheel).
 */
public final class DriveConstants {
  private DriveConstants() {}

  // ---- Chassis physical (estimates; adjust to your real robot) ----
  public static final double ROBOT_MASS_KG  = 50.0;
  public static final double BUMPER_SIZE_M  = 0.7;  // square footprint incl. bumpers
  public static final double TRACK_LENGTH_M = 0.6;  // front<->back wheel distance
  public static final double TRACK_WIDTH_M  = 0.6;  // left<->right wheel distance

  // ---- MK4n module ----
  public static final double WHEEL_COF       = 1.2;
  public static final int    MK4N_GEAR_LEVEL = 2;   // 1 = L1+ (7.13), 2 = L2 (5.9), 3 = L3+ (5.36)

  // ---- Teleop ----
  public static final double JOYSTICK_DEADBAND = 0.1;
}
