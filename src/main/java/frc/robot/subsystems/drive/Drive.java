package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.littletonrobotics.junction.Logger;

/**
 * MK4n swerve drivetrain.
 *
 * <p>Currently SIM-ONLY: it drives a maple-sim {@link SelfControlledSwerveDriveSimulation}, which
 * runs the per-module steer/drive control and full chassis physics internally. There is no hardware
 * IO yet — on a real robot this subsystem is inert (you'll add a real IO path once you have a robot
 * and a Tuner X TunerConstants). Module geometry/ratios come from {@code COTS.ofMark4n}.
 */
public class Drive extends SubsystemBase {

  /** Non-null only in simulation. */
  private final SelfControlledSwerveDriveSimulation sim;

  public Drive() {
    if (RobotBase.isSimulation()) {
      DriveTrainSimulationConfig config =
          DriveTrainSimulationConfig.Default()
              .withRobotMass(Kilograms.of(DriveConstants.ROBOT_MASS_KG))
              .withBumperSize(
                  Meters.of(DriveConstants.BUMPER_SIZE_M), Meters.of(DriveConstants.BUMPER_SIZE_M))
              .withTrackLengthTrackWidth(
                  Meters.of(DriveConstants.TRACK_LENGTH_M), Meters.of(DriveConstants.TRACK_WIDTH_M))
              .withGyro(COTS.ofPigeon2())
              .withSwerveModule(
                  () ->
                      new SwerveModuleSimulation(
                          COTS.ofMark4n(
                              DCMotor.getKrakenX60Foc(1), // drive motor
                              DCMotor.getKrakenX60Foc(1), // steer motor
                              DriveConstants.WHEEL_COF,
                              DriveConstants.MK4N_GEAR_LEVEL)));

      SwerveDriveSimulation swerveSim =
          new SwerveDriveSimulation(config, new Pose2d(3.0, 3.0, new Rotation2d()));
      sim = new SelfControlledSwerveDriveSimulation(swerveSim);
      SimulatedArena.getInstance().addDriveTrainSimulation(swerveSim);
    } else {
      sim = null;
    }
  }

  /**
   * Command the chassis, field-relative. vx/vy in m/s (field frame, +x away from blue wall, +y
   * left), omega in rad/s (CCW+).
   */
  public void runFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    if (sim != null) {
      sim.runChassisSpeeds(fieldRelativeSpeeds, new Translation2d(), true, true);
    }
  }

  public void stop() {
    runFieldRelative(new ChassisSpeeds());
  }

  public double maxLinearSpeedMps() {
    return sim != null ? sim.maxLinearVelocity().in(MetersPerSecond) : 4.0;
  }

  public double maxAngularSpeedRadPerSec() {
    return sim != null ? sim.maxAngularVelocity().in(RadiansPerSecond) : 2.0 * Math.PI;
  }

  public Pose2d getPose() {
    return sim != null ? sim.getOdometryEstimatedPose() : new Pose2d();
  }

  @Override
  public void periodic() {
    if (sim != null) {
      sim.periodic(); // update simulated odometry from the cached module states
      Logger.recordOutput("Drive/OdometryPose", sim.getOdometryEstimatedPose());
      Logger.recordOutput("Drive/ActualSimPose", sim.getActualPoseInSimulationWorld());
      Logger.recordOutput("Drive/MeasuredStates", sim.getMeasuredStates());
    }
  }

  @Override
  public void simulationPeriodic() {
    // Advance the maple-sim physics world once per loop.
    SimulatedArena.getInstance().simulationPeriodic();
  }
}
