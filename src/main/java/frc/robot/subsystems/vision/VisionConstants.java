package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * Tunables for the PhotonVision-based AprilTag localization.
 *
 * <p>The four {@code robotToCameraN} transforms are PLACEHOLDERS — replace them with the real mounting
 * pose of each camera from your CAD once it's finalized. Convention: robot origin at the center of the
 * frame on the floor, +x forward, +y left, +z up. A camera's transform is its lens pose in that frame;
 * {@code Rotation3d(roll, pitch, yaw)} with negative pitch tilting the lens upward.
 */
public final class VisionConstants {
  private VisionConstants() {}

  // AprilTag layout — 2026 "Rebuilt" welded competition field (ships with WPILib 2026).
  public static final AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  // Camera names — must match the camera names configured in the PhotonVision UI on the real robot.
  public static final String camera0Name = "front_left";
  public static final String camera1Name = "front_right";
  public static final String camera2Name = "back_left";
  public static final String camera3Name = "back_right";

  // Robot-to-camera transforms — positions from CAD (mm -> m), converted from the CAD frame to the
  // WPILib robot frame (+x fwd, +y left, +z up) via x_robot=-y_cad, y_robot=-x_cad, z_robot=-z_cad.
  // Rotation convention: Rotation3d(roll, pitch, yaw) in radians; negative pitch tilts the lens UP,
  // positive yaw turns the lens toward +y (left). Roll is 0.
  //
  //  - cam0/cam1: lower FRONT pair (0.346 m high), toed-IN 15deg, tilted UP 15deg.
  //  - cam2/cam3: upper pair (0.470 m high), yawed 120deg to look BACK, tilted UP 30deg.
  public static final Transform3d robotToCamera0 = // front-left, low
      new Transform3d(
          0.31489, 0.13308, 0.34561,
          new Rotation3d(0.0, Math.toRadians(-15.0), Math.toRadians(-15.0)));
  public static final Transform3d robotToCamera1 = // front-right, low
      new Transform3d(
          0.31489, -0.13308, 0.34561,
          new Rotation3d(0.0, Math.toRadians(-15.0), Math.toRadians(15.0)));
  public static final Transform3d robotToCamera2 = // back-left, high
      new Transform3d(
          0.29528, 0.30119, 0.47009,
          new Rotation3d(0.0, Math.toRadians(-30.0), Math.toRadians(120.0)));
  public static final Transform3d robotToCamera3 = // back-right, high
      new Transform3d(
          0.29528, -0.30119, 0.47009,
          new Rotation3d(0.0, Math.toRadians(-30.0), Math.toRadians(-120.0)));

  // Basic filtering thresholds.
  public static final double maxAmbiguity = 0.3; // reject single-tag estimates above this
  public static final double maxZError = 0.75; // reject estimates floating/sinking more than this (m)

  // Standard-deviation baselines, defined for a 1 m average tag distance and a single tag.
  public static final double linearStdDevBaseline = 0.02; // meters
  public static final double angularStdDevBaseline = 0.06; // radians

  // Per-camera trust multipliers (index matches the VisionIO order). Lower = trust this camera more.
  public static final double[] cameraStdDevFactors = new double[] {1.0, 1.0, 1.0, 1.0};

  // MegaTag-2 multipliers (unused with PhotonVision multitag, kept for completeness).
  public static final double linearStdDevMegatag2Factor = 0.5;
  public static final double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY;
}
