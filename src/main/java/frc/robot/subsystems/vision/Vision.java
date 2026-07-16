package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * AprilTag localization. Pulls observations from N cameras (via {@link VisionIO}), filters out bad
 * estimates, scales the measurement trust by tag distance/count/chassis motion, and pushes the
 * survivors into the drivetrain pose estimator through a {@link VisionConsumer}.
 */
public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  public Vision(VisionConsumer consumer, Supplier<ChassisSpeeds> chassisSpeedsSupplier, VisionIO... io) {
    this.consumer = consumer;
    this.chassisSpeedsSupplier = chassisSpeedsSupplier;
    this.io = io;

    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert("Vision camera " + i + " is disconnected.", AlertType.kWarning);
    }
  }

  /** Yaw to the best target seen by the given camera (for aiming commands). */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + i, inputs[i]);
    }

    // Trust shrinks further while the chassis is moving/rotating fast: motion blur, rolling
    // shutter skew, and vision-to-odometry timestamp mismatch all get worse in motion. Linear and
    // angular get INDEPENDENT multipliers (translating fast penalizes only linear trust, spinning
    // fast penalizes only angular trust) -- see the doc on the two factor constants for why a
    // shared multiplier was actively harmful during a fast in-place spin.
    ChassisSpeeds currentSpeeds = chassisSpeedsSupplier.get();
    double speedMetersPerSec =
        Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
    double omegaRadPerSec = Math.abs(currentSpeeds.omegaRadiansPerSecond);
    double linearMotionMultiplier = 1.0 + SPEED_STD_DEV_FACTOR.get() * speedMetersPerSec;
    double angularMotionMultiplier = 1.0 + ROTATION_STD_DEV_FACTOR.get() * omegaRadPerSec;

    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);
      // Built once per camera instead of re-concatenating for every recordOutput call below --
      // this loop runs every 20ms for every camera, so the repeated string allocation adds up.
      String prefix = "Vision/Camera" + cameraIndex;

      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      for (int tagId : inputs[cameraIndex].tagIds) {
        APRIL_TAG_LAYOUT.getTagPose(tagId).ifPresent(tagPoses::add);
      }

      for (var observation : inputs[cameraIndex].poseObservations) {
        boolean noTags = observation.tagCount() == 0;
        boolean highAmbiguity =
            observation.tagCount() == 1 && observation.ambiguity() > MAX_AMBIGUITY.get();
        boolean badZ = Math.abs(observation.pose().getZ()) > MAX_Z_ERROR.get();
        boolean badX =
            observation.pose().getX() < 0.0
                || observation.pose().getX() > APRIL_TAG_LAYOUT.getFieldLength();
        boolean badY =
            observation.pose().getY() < 0.0
                || observation.pose().getY() > APRIL_TAG_LAYOUT.getFieldWidth();
        boolean rejectPose = noTags || highAmbiguity || badZ || badX || badY;

        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
          continue;
        }
        robotPosesAccepted.add(observation.pose());

        // Trust shrinks with distance (squared), grows with tag count, and shrinks further with
        // chassis speed/rotation rate.
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = LINEAR_STD_DEV_BASELINE.get() * stdDevFactor * linearMotionMultiplier;
        double angularStdDev = ANGULAR_STD_DEV_BASELINE.get() * stdDevFactor * angularMotionMultiplier;
        if (cameraIndex < CAMERA_STD_DEV_FACTORS.length) {
          linearStdDev *= CAMERA_STD_DEV_FACTORS[cameraIndex];
          angularStdDev *= CAMERA_STD_DEV_FACTORS[cameraIndex];
        }

        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // AdvantageKit already publishes recordOutput data to NT for live viewing (AdvantageScope/
      // Elastic) AND to the wpilog for replay -- the old per-camera SmartDashboard.put* debug
      // calls here (9 of them, every 20ms, for every camera) were pure duplicates of this data
      // and a real contributor to loop overruns; removed rather than throttled since nothing
      // actually consumed them that recordOutput doesn't already cover.
      Logger.recordOutput(prefix + "/TagPoses", tagPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(prefix + "/RobotPoses", robotPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(prefix + "/RobotPosesAccepted", robotPosesAccepted.toArray(new Pose3d[0]));
      Logger.recordOutput(prefix + "/RobotPosesRejected", robotPosesRejected.toArray(new Pose3d[0]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
  }

  /** Receives accepted vision estimates — wire this to the drivetrain pose estimator. */
  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
