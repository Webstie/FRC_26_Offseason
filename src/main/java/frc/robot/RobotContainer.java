// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.LedsConfig.*;
import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.MatchStateCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.ShootCommands;
import frc.robot.subsystems.carriage.Carriage;
import frc.robot.subsystems.carriage.CarriageIOSim;
import frc.robot.subsystems.carriage.CarriageIOTalonFX;
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
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.RollersIOSim;
import frc.robot.subsystems.rollers.RollersIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.shooter.ShooterProfile;
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
import org.littletonrobotics.junction.Logger;

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
  private final Carriage carriage =
      new Carriage(RobotBase.isReal() ? new CarriageIOTalonFX() : new CarriageIOSim());
  private final Rollers rollers =
      new Rollers(RobotBase.isReal() ? new RollersIOTalonFX() : new RollersIOSim());
  private final Leds leds = new Leds();

  // Drivetrain + vision are built in the constructor because in sim they share a maple-sim world.
  private final Drive drive;
  private final Vision vision;
  private SwerveDriveSimulation driveSimulation = null; // non-null only in simulation

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);
  // Built from AutoBuilder.buildAutoChooser() in the constructor (below), which auto-populates
  // with every PathPlannerAuto found in src/main/deploy/pathplanner/autos/ — must run AFTER Drive's
  // constructor, since that's where AutoBuilder.configure() happens.
  private final SendableChooser<Command> autoChooser;

  // Dashboard override for auto-win detection: "Auto" (default) trusts the FMS game-specific
  // message (see isAutoWin() below); "Force Win"/"Force Lose" let the coach decide manually
  // instead, for bench testing or if the field data ever looks wrong.
  private final SendableChooser<String> autoWinModeChooser = new SendableChooser<>();

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
                drive::getChassisSpeeds,
                new VisionIOPhotonVision(CAMERA_0_NAME, ROBOT_TO_CAMERA_0),
                new VisionIOPhotonVision(CAMERA_1_NAME, ROBOT_TO_CAMERA_1),
                new VisionIOPhotonVision(CAMERA_2_NAME, ROBOT_TO_CAMERA_2),
                new VisionIOPhotonVision(CAMERA_3_NAME, ROBOT_TO_CAMERA_3));
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
                drive::getChassisSpeeds,
                new VisionIOPhotonVisionSim(
                    CAMERA_0_NAME, ROBOT_TO_CAMERA_0, driveSimulation::getSimulatedDriveTrainPose),
                new VisionIOPhotonVisionSim(
                    CAMERA_1_NAME, ROBOT_TO_CAMERA_1, driveSimulation::getSimulatedDriveTrainPose),
                new VisionIOPhotonVisionSim(
                    CAMERA_2_NAME, ROBOT_TO_CAMERA_2, driveSimulation::getSimulatedDriveTrainPose),
                new VisionIOPhotonVisionSim(
                    CAMERA_3_NAME, ROBOT_TO_CAMERA_3, driveSimulation::getSimulatedDriveTrainPose));
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
                drive::getChassisSpeeds,
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {});
      }
    }

    // Must run BEFORE buildAutoChooser() below: that call loads every PathPlannerAuto from
    // src/main/deploy/pathplanner/autos/ and resolves each path's event markers against this
    // registry immediately, so any name referenced by a marker has to already be registered here.
    // DeployIntake and Prespin are both self-finishing/non-blocking and don't require Drive, so
    // they're safe on a mid-path event marker (run concurrently with that path's following
    // command). Shoot DOES require Drive (it aims the chassis at the goal) -- it needs to run AFTER
    // a path finishes, not during one. Prespin also requires Shooter, same as Shoot -- drop a
    // Prespin marker early in a path and a Shoot marker/step later, and scheduling Shoot
    // automatically cancels Prespin and takes over (no explicit hand-off needed). See AutoCommands'
    // javadoc for each.
    NamedCommands.registerCommand(
        "Shoot", AutoCommands.shoot(drive, shooter, carriage, rollers, indexer, intake));
    NamedCommands.registerCommand("DeployIntake", AutoCommands.deployIntake(carriage, intake));
    NamedCommands.registerCommand("Prespin", AutoCommands.prespin(drive, shooter));

    // Must run after Drive's constructor (AutoBuilder.configure() happens there) — populates with
    // every PathPlannerAuto found in src/main/deploy/pathplanner/autos/, "None" as the default.
    autoChooser = AutoBuilder.buildAutoChooser();

    // Warm-up: force the AprilTag field layout to load NOW, at boot. Loading it
    // (AprilTagFieldLayout.loadField -> a large JSON parsed through Jackson, a ~1-2 s one-time cost
    // on the roboRIO) otherwise happens lazily the first time FieldConstants.goal() runs — which is
    // inside the auto-shoot command, so it landed on the first button press mid-match. Touching it
    // here moves that cost into startup/disabled. (Vision is no-op on the real robot right now, so
    // nothing else loads the layout at boot.)
    FieldConstants.goal();

    autoWinModeChooser.setDefaultOption("Auto (FMS)", "AUTO");
    autoWinModeChooser.addOption("Force Win", "WIN");
    autoWinModeChooser.addOption("Force Lose", "LOSE");
    SmartDashboard.putData("AutoWin Mode", autoWinModeChooser);

    // Starts once on the rising edge of teleop enabled. Deliberately NOT autonomous-enabled: that
    // trigger only fires when a real match/Practice-mode auto period actually runs, so a driver
    // practicing by jumping straight to the Teleoperated tab on the DS never fired it at all. The
    // shift schedule only matters in teleop anyway, so starting the clock there covers both cases.
    new Trigger(DriverStation::isTeleopEnabled).onTrue(new MatchStateCommand(this::isAutoWin));

    configureAutos();
    configureBindings();
  }

  /**
   * Did OUR alliance score more Fuel in auto (i.e. does our HUB stay active during Shift1)?
   *
   * <p>Governed by the "AutoWin Mode" chooser on the dashboard: "Force Win"/"Force Lose" let the
   * coach decide directly (bench testing, or overriding the field if its data ever looks wrong).
   * The default, "Auto (FMS)", trusts the FMS-provided game-specific message: the 2026 field sends
   * a single {@code 'R'}/{@code 'B'} character naming the alliance whose HUB goes inactive first,
   * ~3s after auto ends ({@link DriverStation#getGameSpecificMessage()} — see the 2026 Game Manual
   * / WPILib game-data docs). That message is empty until the field sends it (before the match,
   * and for a few seconds after auto) and also on a bench/sim robot with no Driver Station game
   * data set — in both cases, while still in "Auto" mode, we default to false ("lose") until real
   * data arrives.
   */
  private boolean isAutoWin() {
    String mode = autoWinModeChooser.getSelected();
    if ("WIN".equals(mode)) {
      return true;
    }
    if ("LOSE".equals(mode)) {
      return false;
    }

    String gameData = DriverStation.getGameSpecificMessage();
    var alliance = DriverStation.getAlliance();
    if (gameData != null && !gameData.isEmpty() && alliance.isPresent()) {
      boolean redInactiveFirst = gameData.charAt(0) == 'R';
      boolean weAreRed = alliance.get() == Alliance.Red;
      return redInactiveFirst != weAreRed;
    }
    return false;
  }

  /**
   * True once the robot is physically past the goal (ported from 25-26-swerve's hub-relative
   * switch, {@code RobotContainer.java}'s inline {@code hubX} check there) — at that point shooting
   * at the goal means turning back toward your own wall, so the right trigger should feed to a
   * corner ({@link ShootCommands#feed}) instead ({@link ShootCommands#autoShoot}). Mirrored for
   * alliance via {@link FieldConstants#goal()}: blue's X increases away from its own wall, red's
   * decreases, so "past" flips direction accordingly.
   */
  private boolean pastFeedZone() {
    double x = drive.getPose().getX();
    double goalX = FieldConstants.goal().getX();
    boolean pastZone = FieldConstants.isRedAlliance() ? x < goalX : x > goalX;
    // Logged so a "feed never triggers" report on the field can be diagnosed from AdvantageScope
    // alone: if PoseX never crosses goalX even while the robot is physically past it, the pose
    // estimator (not this switch logic) is the thing to fix.
    Logger.recordOutput("RobotContainer/PastFeedZone/PoseX", x);
    Logger.recordOutput("RobotContainer/PastFeedZone/GoalX", goalX);
    Logger.recordOutput("RobotContainer/PastFeedZone/IsRedAlliance", FieldConstants.isRedAlliance());
    Logger.recordOutput("RobotContainer/PastFeedZone/Result", pastZone);
    return pastZone;
  }

  private void configureAutos() {
    // "None" plus every PathPlannerAuto were already added by AutoBuilder.buildAutoChooser() above;
    // these are extra bench-testing options layered onto the same chooser.
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Same actions are registered as NamedCommands above (for PathPlanner event markers); these
    // entries just let you bench-test each one standalone from the dashboard.
    autoChooser.addOption(
        "Test: Auto Shoot", AutoCommands.shoot(drive, shooter, carriage, rollers, indexer, intake));
    autoChooser.addOption("Test: Auto Deploy Intake", AutoCommands.deployIntake(carriage, intake));
    autoChooser.addOption("Test: Auto Prespin", AutoCommands.prespin(drive, shooter));

    SmartDashboard.putData("Auto Choices", autoChooser);
  }

  private void configureBindings() {
    // Default teleop drive: left stick translates (field-relative), right stick rotates.
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            m_driverController::getLeftY,
            m_driverController::getLeftX,
            m_driverController::getRightX));

    // POV right: seed the TRUE absolute (blue-frame) heading -- e.g. square the robot against a known
    // field reference and press this to correct gyro drift. Deliberately NOT alliance-flipped: this
    // sets the actual odometry rotation that auto-aim/PathPlanner read, so it must stay in the real
    // blue-origin frame regardless of which alliance you're on. Driver-perspective ("which way feels
    // like forward on the stick") is handled separately in DriveCommands.joystickDrive and never
    // touches this value.
    m_driverController
        .povRight()
        .onTrue(
            Commands.runOnce(
                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero))));

    // --- Temporary bench-tuning window, on the CO-DRIVER controller (port 1) -----------------------
    // (remove once ShooterProfile.TABLE is re-calibrated)
    // Live-shifts the WHOLE distance-interpolated ShooterProfile curve so a systematic long/short or
    // high/low miss can be corrected on the field without redeploying: POV up/down nudge the hood
    // curve, Start/Back nudge the speed curve, POV left resets both offsets to zero.
    //
    // Force ShooterProfile's static init (and its first NT/Elastic publish) to run NOW instead of
    // waiting for the first POV press or the first shot -- same reason ShootCommands.manualShoot()
    // eagerly touches its own tunables once outside the lambda.
    ShooterProfile.resetOffsets();
    m_operatorController
        .povUp()
        .onTrue(Commands.runOnce(() -> ShooterProfile.adjustHoodOffset(0.05)));
    m_operatorController
        .povDown()
        .onTrue(Commands.runOnce(() -> ShooterProfile.adjustHoodOffset(-0.05)));
    m_operatorController
        .start()
        .onTrue(Commands.runOnce(() -> ShooterProfile.adjustSpeedOffset(0.5)));
    m_operatorController
        .back()
        .onTrue(Commands.runOnce(() -> ShooterProfile.adjustSpeedOffset(-0.5)));
    m_operatorController.povLeft().onTrue(Commands.runOnce(ShooterProfile::resetOffsets));

    // A: toggle intake deploy. Deploying (out) auto-starts the rollers; retracting (in) auto-stops
    // them.
    m_driverController
        .leftBumper()
        .onTrue(
            Commands.either(
                Commands.parallel(carriage.retractCommand(), intake.stopCommand()),
                Commands.parallel(carriage.deployCommand(), intake.runCommand()),
                carriage::isDeployed));

    // B: toggle intake roller spin by itself, independent of deploy state.
    m_driverController
        .rightBumper()
        .onTrue(Commands.either(intake.stopCommand(), intake.runCommand(), intake::isRunning));

    // Right trigger held: goal shot, UNLESS the robot is far enough across the field (see
    // pastFeedZone()) that it's closer to a feed corner than the goal — then it's a corner-feed shot
    // instead (ported from 25-26-swerve). Both pre-spin the flywheel + adjust the hood from distance,
    // turn the chassis to face the target (driver keeps field-relative translation), then once
    // aligned + at speed + at angle, feed while slowly pulling the carriage back. Red/blue handled in
    // FieldConstants.goal()/nearestFeedCorner().
    //
    // Deliberately two separate Trigger bindings (rightTrigger().and(pastFeedZone)) instead of a
    // single Commands.either(feed, autoShoot, this::pastFeedZone): ConditionalCommand/Commands.either
    // only checks its condition ONCE, when the command is scheduled, and never again for the rest of
    // that hold -- so a driver holding the trigger while driving from the near half across into the
    // feed zone (e.g. through the trench into midfield) would get stuck running whichever command was
    // picked at the moment they first pressed it. Two Triggers ANDed with pastFeedZone() re-evaluate
    // it every loop, so crossing the boundary mid-hold correctly cancels one command and schedules
    // the other.
    m_driverController
        .rightTrigger()
        .and(this::pastFeedZone)
        .whileTrue(
            ShootCommands.feed(
                drive,
                shooter,
                carriage,
                rollers,
                indexer,
                intake,
                m_driverController::getLeftY,
                m_driverController::getLeftX));
    m_driverController
        .rightTrigger()
        .and(() -> !pastFeedZone())
        .whileTrue(
            ShootCommands.autoShoot(
                drive,
                shooter,
                carriage,
                rollers,
                indexer,
                intake,
                m_driverController::getLeftY,
                m_driverController::getLeftX));

    // Left trigger held: manual shot — no vision, no auto-aim. Same feed choreography as the
    // auto-shoot but at the fixed setpoints under /Tuning/ManualShoot/*, and the chassis is left
    // fully under driver control. Tune the preset speed/hood live in AdvantageScope.
    m_driverController
        .leftTrigger()
        .whileTrue(ShootCommands.manualShoot(drive, shooter, carriage, rollers, indexer, intake));

    // Y held: "outtake everything" — reverse intake/rollers/indexer/shooter and deploy the carriage
    // out to clear a stuck game piece. Release stops the rollers/indexer/shooter, but the carriage
    // stays deployed (not retracted).
    m_driverController
        .y()
        .whileTrue(new OuttakeCommand(intake, carriage, rollers, indexer, shooter));

    // Status LEDs: one color per driver action, highest priority first (first true wins); falls
    // back to the alliance color when no action is active. See Constants.LedsConfig for the colors.
    leds.addState(m_driverController.rightTrigger(), COLOR_AUTO_SHOOT);
    leds.addState(m_driverController.leftTrigger(), COLOR_MANUAL_SHOOT);
    leds.addState(m_driverController.y(), COLOR_OUTTAKE);
    leds.addState(intake::isRunning, COLOR_INTAKE_SPIN);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
