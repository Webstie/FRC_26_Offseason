package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/**
 * Simulated AprilTag camera. Extends the real IO (so the frame-parsing logic is identical) and adds a
 * PhotonVision {@link VisionSystemSim} that renders what each camera would see. It is driven by the
 * robot's GROUND-TRUTH pose from maple-sim (not the estimated pose), so the resulting vision estimates
 * genuinely correct the wheel-odometry drift.
 */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
  // One shared simulated vision world for all cameras (holds the field tag layout).
  private static VisionSystemSim visionSim;

  private final Supplier<Pose2d> poseSupplier;
  private final PhotonCameraSim cameraSim;

  public VisionIOPhotonVisionSim(
      String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
    super(name, robotToCamera);
    this.poseSupplier = poseSupplier;

    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(aprilTagLayout);
    }

    var cameraProperties = new SimCameraProperties();
    cameraProperties.setCalibration(960, 720, Rotation2d.fromDegrees(90.0)); // res + diagonal FOV
    cameraProperties.setCalibError(0.35, 0.10); // avg + stddev reprojection error (px)
    // 50 FPS matches the 50 Hz robot loop so (almost) every loop gets a fresh frame. At a lower FPS
    // (e.g. 30) the camera only produces a frame every ~1.6 loops, so the logged tag list goes empty
    // on the in-between loops and the AdvantageScope vision lines flicker. Raise this to de-flicker.
    cameraProperties.setFPS(50.0);
    cameraProperties.setAvgLatencyMs(20.0);
    cameraProperties.setLatencyStdDevMs(3.0);
    cameraSim = new PhotonCameraSim(camera, cameraProperties);
    visionSim.addCamera(cameraSim, robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    visionSim.update(poseSupplier.get());
    super.updateInputs(inputs);
  }
}
