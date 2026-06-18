package frc.robot.commands;

import static frc.robot.Constants.IndexerConfig.INDEXER_VELOCITY;
import static frc.robot.Constants.IntakeConfig.INTAKE_DEPLOY_DOWN_POSITION;
import static frc.robot.Constants.IntakeConfig.INTAKE_DEPLOY_UP_POSITION;
import static frc.robot.Constants.IntakeConfig.INTAKE_VELOCITY;
import static frc.robot.Constants.RollersConfig.ROLLERS_VELOCITY;
import static frc.robot.Constants.ShooterConfig.SHOOTER_VELOCITY;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Rollers;
import frc.robot.subsystems.Shooter;

/** Reverses intake, rollers, indexer, and shooter together to spit a game piece back out. */
public class OuttakeCommand extends ParallelCommandGroup {

    public OuttakeCommand(Intake intake, Rollers rollers, Indexer indexer, Shooter shooter) {
        addCommands(
            intake.startEnd(
                () -> {
                    intake.setDeployMotorPosition(INTAKE_DEPLOY_DOWN_POSITION);
                    intake.setIntakeMotorVelocity(-INTAKE_VELOCITY);
                },
                () -> {
                    intake.setIntakeMotorVelocity(0);
                    intake.setDeployMotorPosition(INTAKE_DEPLOY_UP_POSITION);
                }
            ),
            rollers.startEnd(
                () -> rollers.setRollersMotorVelocity(-ROLLERS_VELOCITY),
                () -> rollers.setRollersMotorVelocity(0)
            ),
            indexer.startEnd(
                () -> indexer.setIndexerMotorVelocity(-INDEXER_VELOCITY),
                () -> indexer.setIndexerMotorVelocity(0)
            ),
            shooter.startEnd(
                () -> shooter.setShooterMotorVelocity(-SHOOTER_VELOCITY),
                () -> shooter.setShooterMotorVelocity(0)
            )
        );
    }
}
