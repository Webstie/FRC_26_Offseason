package frc.robot.commands;

import static frc.robot.Constants.IndexerConfig.INDEXER_VELOCITY;
import static frc.robot.Constants.RollersConfig.ROLLERS_VELOCITY;
import static frc.robot.Constants.ShooterConfig.SHOOTER_VELOCITY;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.shooter.Shooter;

/** Runs shooter, indexer, and rollers together. All stop when the command ends. */
public class ShootingCommand extends ParallelCommandGroup {

    public ShootingCommand(Shooter shooter, Indexer indexer, Rollers rollers) {
        addCommands(
            shooter.startEnd(
                () -> shooter.setShooterMotorVelocity(SHOOTER_VELOCITY),
                () -> shooter.setShooterMotorVelocity(0)
            ),
            indexer.startEnd(
                () -> indexer.setIndexerMotorVelocity(INDEXER_VELOCITY),
                () -> indexer.setIndexerMotorVelocity(0)
            ),
            rollers.startEnd(
                () -> rollers.setRollersMotorVelocity(ROLLERS_VELOCITY),
                () -> rollers.setRollersMotorVelocity(0)
            )
        );
    }
}
