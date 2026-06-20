// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.IntakeConfig.*;
import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.ShootingCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.RollersIOSim;
import frc.robot.subsystems.rollers.RollersIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Superstructure subsystems (real vs sim chosen per IO).
  private final Indexer indexer =
      new Indexer(RobotBase.isReal() ? new IndexerIOTalonFX() : new IndexerIOSim());
  private final Shooter shooter =
      new Shooter(RobotBase.isReal() ? new ShooterIOTalonFX() : new ShooterIOSim());
  private final Intake intake =
      new Intake(RobotBase.isReal() ? new IntakeIOTalonFX() : new IntakeIOSim());
  private final Rollers rollers =
      new Rollers(RobotBase.isReal() ? new RollersIOTalonFX() : new RollersIOSim());

  // Drivetrain + vision are built in the constructor because in sim they share a maple-sim world.
  private final Drive drive;
  private final Vision vision;
  private SwerveDriveSimulation driveSimulation = null; // non-null only in simulation

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL -> {
        // Real hardware: TalonFX modules + Pigeon2 + PhotonVision coprocessors.
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(camera0Name, robotToCamera0),
                new VisionIOPhotonVision(camera1Name, robotToCamera1),
                new VisionIOPhotonVision(camera2Name, robotToCamera2),
                new VisionIOPhotonVision(camera3Name, robotToCamera3));
      }
      case SIM -> {
        // Build the maple-sim physics world, then drive + vision both read from it.
        DriveTrainSimulationConfig simConfig =
            DriveTrainSimulationConfig.Default()
                .withRobotMass(Kilograms.of(DriveConstants.ROBOT_MASS_KG))
                .withBumperSize(
                    Meters.of(DriveConstants.BUMPER_SIZE_M), Meters.of(DriveConstants.BUMPER_SIZE_M))
                .withCustomModuleTranslations(Drive.getModuleTranslations())
                .withGyro(COTS.ofPigeon2())
                // Build the sim module from TunerConstants so the maple-sim physics wheel radius and
                // gear ratios MATCH the odometry/control path (Module.java uses TunerConstants.WheelRadius
                // + gear ratios). Using COTS.ofMark4n instead would hardcode a 2.0in wheel and desync the
                // sim odometry from the physics. This auto-tracks your real Tuner X file once pasted.
                .withSwerveModule(
                    () ->
                        new SwerveModuleSimulation(
                            new SwerveModuleSimulationConfig(
                                DCMotor.getKrakenX60Foc(1), // drive motor
                                DCMotor.getKrakenX60Foc(1), // steer motor
                                TunerConstants.FrontLeft.DriveMotorGearRatio,
                                TunerConstants.FrontLeft.SteerMotorGearRatio,
                                Volts.of(0.2), // drive friction voltage (sim realism)
                                Volts.of(0.2), // steer friction voltage (sim realism)
                                Meters.of(TunerConstants.FrontLeft.WheelRadius),
                                KilogramSquareMeters.of(0.03), // steer rotational inertia
                                DriveConstants.WHEEL_COF)));
        driveSimulation =
            new SwerveDriveSimulation(simConfig, new Pose2d(3.0, 3.0, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOSim(driveSimulation.getModules()[0]),
                new ModuleIOSim(driveSimulation.getModules()[1]),
                new ModuleIOSim(driveSimulation.getModules()[2]),
                new ModuleIOSim(driveSimulation.getModules()[3]));
        drive.setPose(driveSimulation.getSimulatedDriveTrainPose());

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
                new VisionIOPhotonVisionSim(
                    camera1Name, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose),
                new VisionIOPhotonVisionSim(
                    camera2Name, robotToCamera2, driveSimulation::getSimulatedDriveTrainPose),
                new VisionIOPhotonVisionSim(
                    camera3Name, robotToCamera3, driveSimulation::getSimulatedDriveTrainPose));
      }
      default -> {
        // Replay: empty IO implementations (data comes from the log).
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {});
      }
    }

    configureBindings();
  }

  private void configureBindings() {
    // Default teleop drive: left stick translates (field-relative), right stick rotates.
    drive.setDefaultCommand(
        drive.run(
            () -> {
              double maxV = drive.getMaxLinearSpeedMetersPerSec();
              double maxOmega = drive.getMaxAngularSpeedRadPerSec();
              double vx =
                  -MathUtil.applyDeadband(
                          m_driverController.getLeftY(), DriveConstants.JOYSTICK_DEADBAND)
                      * maxV;
              double vy =
                  -MathUtil.applyDeadband(
                          m_driverController.getLeftX(), DriveConstants.JOYSTICK_DEADBAND)
                      * maxV;
              double omega =
                  -MathUtil.applyDeadband(
                          m_driverController.getRightX(), DriveConstants.JOYSTICK_DEADBAND)
                      * maxOmega;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, drive.getRotation()));
            }));

    // A held: extend the intake (deploy down) + spin rollers; release: stop + retract.
    m_driverController
        .a()
        .whileTrue(
            intake.startEnd(
                () -> {
                  intake.setDeployMotorPosition(INTAKE_DEPLOY_DOWN_POSITION);
                  intake.setIntakeMotorVelocity(INTAKE_VELOCITY);
                },
                () -> {
                  intake.setIntakeMotorVelocity(0);
                  intake.setDeployMotorPosition(INTAKE_DEPLOY_UP_POSITION);
                }));

    // Right trigger runs the full shooting sequence while held
    // m_driverController.rightTrigger().whileTrue(new ShootingCommand(shooter, indexer, rollers));

    // Left trigger runs the outtake (everything reversed) while held
    // m_driverController.leftTrigger().whileTrue(new OuttakeCommand(intake, rollers, indexer, shooter));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
