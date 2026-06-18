// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.IntakeConfig.INTAKE_VELOCITY;

import frc.robot.subsystems.*;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.ShootingCommand;
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
  private final Indexer indexer = new Indexer();
  private final Shooter shooter = new Shooter();
  private final Intake intake = new Intake();
  private final Rollers rollers = new Rollers();

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
    // A button toggles the intake on/off
    m_driverController.a().toggleOnTrue(
        intake.startEnd(
            () -> intake.setIntakeMotorVelocity(INTAKE_VELOCITY),
            () -> intake.setIntakeMotorVelocity(0)
        )
    );

    // Right trigger runs the full shooting sequence while held
    m_driverController.rightTrigger().whileTrue(new ShootingCommand(shooter, indexer, rollers));

    // Left trigger runs the outtake (everything reversed) while held
    m_driverController.leftTrigger().whileTrue(new OuttakeCommand(intake, rollers, indexer, shooter));
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
