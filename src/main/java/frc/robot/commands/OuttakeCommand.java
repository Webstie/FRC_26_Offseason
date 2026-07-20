package frc.robot.commands;

import static frc.robot.Constants.IndexerConfig.INDEXER_VELOCITY;
import static frc.robot.Constants.RollersConfig.ROLLERS_VELOCITY;
import static frc.robot.Constants.ShooterConfig.SHOOTER_OUTTAKE_VELOCITY;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.carriage.Carriage;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.shooter.Shooter;

/**
 * "Outtake everything" — for clearing a stuck game piece. Deploys the carriage and runs the intake
 * rollers, indexer, rollers, and shooter all in reverse so a jammed piece is spit back out. On
 * release the rollers/indexer/shooter stop, but the carriage stays deployed (not retracted) so the
 * intake is left out.
 */
public class OuttakeCommand extends ParallelCommandGroup {

    public OuttakeCommand(
        Intake intake, Carriage carriage, Rollers rollers, Indexer indexer, Shooter shooter) {
        addCommands(
            carriage.runOnce(carriage::deploy),
            intake.startEnd(intake::spinReverse, intake::stop),
            rollers.startEnd(
                () -> rollers.setRollersMotorVelocity(-ROLLERS_VELOCITY.get()),
                () -> rollers.setRollersMotorVelocity(0)
            ),
            indexer.startEnd(
                () -> indexer.setIndexerMotorVelocity(-INDEXER_VELOCITY.get()),
                () -> indexer.setIndexerMotorVelocity(0)
            ),
            shooter.startEnd(
                () -> shooter.setShooterMotorVelocity(-SHOOTER_OUTTAKE_VELOCITY.get()),
                () -> shooter.stopShooter()
            )
        );
    }
}
