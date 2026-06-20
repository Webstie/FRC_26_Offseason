// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.IntakeConfig.*;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
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
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.ShootingCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Indexer indexer = new Indexer(
      RobotBase.isReal() ? new IndexerIOTalonFX() : new IndexerIOSim());
  private final Shooter shooter = new Shooter(
      RobotBase.isReal() ? new ShooterIOTalonFX() : new ShooterIOSim());
  private final Intake intake = new Intake(
      RobotBase.isReal() ? new IntakeIOTalonFX() : new IntakeIOSim());
  private final Rollers rollers = new Rollers(
      RobotBase.isReal() ? new RollersIOTalonFX() : new RollersIOSim());
  private final Drive drive = new Drive();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Default teleop drive: left stick translates (field-relative), right stick rotates.
    drive.setDefaultCommand(
        drive.run(
            () -> {
              double maxV = drive.maxLinearSpeedMps();
              double maxOmega = drive.maxAngularSpeedRadPerSec();
              double vx = -MathUtil.applyDeadband(m_driverController.getLeftY(), DriveConstants.JOYSTICK_DEADBAND) * maxV;
              double vy = -MathUtil.applyDeadband(m_driverController.getLeftX(), DriveConstants.JOYSTICK_DEADBAND) * maxV;
              double omega = -MathUtil.applyDeadband(m_driverController.getRightX(), DriveConstants.JOYSTICK_DEADBAND) * maxOmega;
              drive.runFieldRelative(new ChassisSpeeds(vx, vy, omega));
            }));

    // A button: each press flips intakePitchPositionFlag, then runs one of two transient actions.
    // m_driverController.a().onTrue(
    //     intake.changePitchPositionCommand()
    //         .andThen(Commands.either(
    //             Commands.runOnce(() -> {
    //                 intake.setDeployMotorPosition(INTAKE_DEPLOY_DOWN_POSITION);
    //                 intake.setIntakeMotorVelocity(INTAKE_VELOCITY);
    //             }, intake),
    //             Commands.runOnce(() -> {
    //                 intake.setIntakeMotorVelocity(0);
    //                 intake.setDeployMotorPosition(INTAKE_DEPLOY_UP_POSITION);
    //             }, intake),
    //             intake::getIntakePitchFlag
    //         ))
    // );
      // A held: extend the intake (deploy down) + spin rollers; release: stop + retract.
      m_driverController.a().whileTrue(
      intake.startEnd(
          () -> {
              intake.setDeployMotorPosition(INTAKE_DEPLOY_DOWN_POSITION);
              intake.setIntakeMotorVelocity(INTAKE_VELOCITY);
          },
          () -> {
              intake.setIntakeMotorVelocity(0);
              intake.setDeployMotorPosition(INTAKE_DEPLOY_UP_POSITION);
          }
      )
  );

    // Right trigger runs the full shooting sequence while held
    // m_driverController.rightTrigger().whileTrue(new ShootingCommand(shooter, indexer, rollers));

    // // Left trigger runs the outtake (everything reversed) while held
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
