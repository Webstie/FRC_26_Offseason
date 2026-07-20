package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.util.LoggedTunableNumber;

/**
 * Tunables for the PhotonVision-based AprilTag localization.
 *
 * <p>The four {@code ROBOT_TO_CAMERA_N} transforms are PLACEHOLDERS — replace them with the real
 * mounting pose of each camera from your CAD once it's finalized. Convention: robot origin at the
 * center of the frame on the floor, +x forward, +y left, +z up. A camera's transform is its lens
 * pose in that frame; {@code Rotation3d(roll, pitch, yaw)} with negative pitch tilting the lens
 * upward.
 */
public final class VisionConstants {
  private VisionConstants() {}

  // AprilTag layout — 2026 "Rebuilt" welded competition field (ships with WPILib 2026).
  public static final AprilTagFieldLayout APRIL_TAG_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  // Camera names — must match the camera names configured in the PhotonVision UI on the real robot.
  public static final String CAMERA_0_NAME = "leftfront";
  public static final String CAMERA_1_NAME = "leftback";
  public static final String CAMERA_2_NAME = "rightfront";
  public static final String CAMERA_3_NAME = "rightback";

  // Robot-to-camera transforms - placeholder lens poses in the
  // WPILib robot frame (+x fwd, +y left, +z up). Replace these placeholders with real measured
  // lens positions from the robot/CAD.
  // Rotation convention: Rotation3d(roll, pitch, yaw) in radians. Negative pitch tilts the lens UP;
  // yaw 0deg faces forward, yaw 180deg faces backward. Roll is 0 for all placeholders.
  public static final Transform3d ROBOT_TO_CAMERA_0 = // leftfront placeholder
      new Transform3d(
          -0.331, 0.151, 0.475,
          new Rotation3d(0.0, Math.toRadians(-25), Math.toRadians(55.0)));
  public static final Transform3d ROBOT_TO_CAMERA_2 = // rightfront placeholder
      new Transform3d(
          -0.331, -0.151, 0.475,
          new Rotation3d(0.0, Math.toRadians(-25), Math.toRadians(-55)));
  public static final Transform3d ROBOT_TO_CAMERA_1 = // leftback placeholder
     new Transform3d(
          -0.325, 0.151, 0.373,
          new Rotation3d(0.0, Math.toRadians(-30.0), Math.toRadians(-165.0)));
  public static final Transform3d ROBOT_TO_CAMERA_3 = // rightback placeholder
      new Transform3d(
          -0.325, -0.151, 0.373,
          new Rotation3d(0.0, Math.toRadians(-30.0), Math.toRadians(165.0)));

  // Basic filtering thresholds — tunable live under "Tuning/Vision/*" (Constants.TUNING_MODE).
  public static final LoggedTunableNumber MAX_AMBIGUITY =
      new LoggedTunableNumber("Vision/MaxAmbiguity", 0.3); // reject single-tag estimates above this
  public static final LoggedTunableNumber MAX_Z_ERROR =
      new LoggedTunableNumber("Vision/MaxZErrorMeters", 0.75); // reject estimates floating/sinking more than this
  // Standard-deviation baselines, defined for a 1 m average tag distance and a single tag.
  // TEMP 10x bump (0.02->0.2, 0.06->0.6) to test the close-range-overtrust jitter theory: sim spawns
  // the robot a few meters from tags, so stdDevFactor = distance^2/tagCount was landing small enough
  // that each frame's noisy PhotonVisionSim solve got fused in at near-full trust, visibly hopping
  // the estimated pose every ~20ms even at rest. If bumping this makes the idle pose jitter in
  // AdvantageScope go away, that confirms it and these should come back down to a tuned middle
  // ground (not necessarily all the way back to 0.02/0.06) rather than staying at 10x. Both are
  // LoggedTunableNumbers (TUNING_MODE=true) so this can also just be dragged live in Elastic under
  // "Tuning/Vision/*" without a redeploy -- only bumping the code default here so it survives a sim
  // restart during testing.
  public static final LoggedTunableNumber LINEAR_STD_DEV_BASELINE =
      new LoggedTunableNumber("Vision/LinearStdDevBaseline", 0.2); // meters
  public static final LoggedTunableNumber ANGULAR_STD_DEV_BASELINE =
      new LoggedTunableNumber("Vision/AngularStdDevBaseline", 0.6); // radians

  // Trust degrades further while the chassis translates/rotates fast (motion blur, rolling shutter
  // skew, and vision-to-odometry timestamp mismatch all worsen in motion). Kept as TWO SEPARATE
  // multipliers (linear std dev scaled only by |v|, angular only by |omega|) rather than one shared
  // multiplier applied to both: a fast in-place spin has a legitimately worse angular solve (motion
  // blur on the tag corners), but there's no reason it should also tank the X/Y trust -- translation
  // is exactly what's needed to correct the wheel-odometry scrub/slip that a fast spin causes, so
  // gutting it at the same time only makes that drift worse. (Coupling the two was the bug behind an
  // earlier test where spinning in place a few times made the estimated position "fall apart": high
  // omega inflated BOTH std devs together, so vision could barely correct translation right when
  // odometry needed it most.)
  public static final LoggedTunableNumber SPEED_STD_DEV_FACTOR =
      new LoggedTunableNumber("Vision/SpeedStdDevFactor", 0.25); // per m/s, linear std dev only
  public static final LoggedTunableNumber ROTATION_STD_DEV_FACTOR =
      new LoggedTunableNumber("Vision/RotationStdDevFactor", 0.5); // per rad/s, angular std dev only

  // Per-camera trust multipliers (index matches the VisionIO order). Lower = trust this camera more.
  public static final double[] CAMERA_STD_DEV_FACTORS = new double[] {1.0, 1.0, 1.0, 1.0};
}
