package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

/** Real-hardware AprilTag camera using a PhotonVision coprocessor. */
public class VisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;
  private final PhotonPoseEstimator poseEstimator;

  public VisionIOPhotonVision(String name, Transform3d robotToCamera) {
    this.camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
    this.poseEstimator = new PhotonPoseEstimator(VisionConstants.APRIL_TAG_LAYOUT, robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();

    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    // Skip polling a camera we already know is offline: getAllUnreadResults() runs PhotonLib's
    // internal verifyVersion() check, which on a not-found coprocessor reports a full stack-trace
    // error to the DriverStation every call — expensive enough on the RIO to blow the 20ms loop
    // budget when a coprocessor is disconnected/still booting. isConnected() is a cheap NT read.
    if (inputs.connected) {
      for (var result : camera.getAllUnreadResults()) {
        // Latest single-target angle (for aiming).
        if (result.hasTargets()) {
          inputs.latestTargetObservation =
              new TargetObservation(
                  Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                  Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
        } else {
          inputs.latestTargetObservation =
              new TargetObservation(new Rotation2d(), new Rotation2d());
        }

        for (var target : result.targets) {
          if (target.getFiducialId() >= 0) {
            tagIds.add((short) target.getFiducialId());
          }
        }

        var estimatedPose =
            poseEstimator
                .estimateCoprocMultiTagPose(result)
                .or(() -> poseEstimator.estimateLowestAmbiguityPose(result));
        if (estimatedPose.isPresent()) {
          double totalTagDistance = 0.0;
          for (var target : result.targets) {
            totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
          }
          int tagCount = estimatedPose.get().targetsUsed.size();

          poseObservations.add(
              new PoseObservation(
                  result.getTimestampSeconds(),
                  estimatedPose.get().estimatedPose,
                  getPoseAmbiguity(estimatedPose.get()),
                  tagCount,
                  totalTagDistance / tagCount,
                  PoseObservationType.PHOTONVISION));
        }
      }
    }

    inputs.poseObservations = poseObservations.toArray(new PoseObservation[0]);

    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (short id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }

  private double getPoseAmbiguity(EstimatedRobotPose estimatedPose) {
    if (estimatedPose.strategy == PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR) {
      return 0.0;
    }

    double lowestAmbiguity = Double.POSITIVE_INFINITY;
    for (var target : estimatedPose.targetsUsed) {
      double ambiguity = target.getPoseAmbiguity();
      if (ambiguity >= 0.0 && ambiguity < lowestAmbiguity) {
        lowestAmbiguity = ambiguity;
      }
    }
    return Double.isFinite(lowestAmbiguity) ? lowestAmbiguity : 1.0;
  }
}
