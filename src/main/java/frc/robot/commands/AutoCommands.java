// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.ShootSequenceConfig.BENCH_SHOOTER_RPS;
import static frc.robot.Constants.ShootSequenceConfig.TARGET_HOOD_ROTATIONS;
import static frc.robot.Constants.ShootSequenceConfig.USE_DISTANCE_PROFILE;
import static frc.robot.Constants.ShooterConfig.HOOD_MAX_ROTATIONS;
import static frc.robot.Constants.ShooterConfig.HOOD_MIN_ROTATIONS;
import static frc.robot.Constants.ShooterConfig.HOOD_REST_ROTATIONS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.subsystems.carriage.Carriage;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterProfile;

/**
 * Discrete, self-finishing actions for autonomous sequences. Unlike the teleop {@link
 * ShootCommands} factories (bound with {@code whileTrue}, run for as long as a button is held),
 * these end on their own so they can sit in a {@code Commands.sequence(...)} or a PathPlanner auto.
 */
public final class AutoCommands {
  private AutoCommands() {}

  /**
   * Autonomous shot: the SAME aim + distance-interpolated speed/hood as the teleop right-trigger
   * shot ({@link ShootCommands#autoShoot}) -- no fixed presets, since those aren't accurate once
   * the shooting spot varies at all. Requires {@code Drive} (it turns the chassis to face the
   * goal), self-terminates once the shot completes, and does NOT re-deploy the carriage afterward
   * -- see {@code ShootCommands.autoShoot}'s {@code forAutonomous} javadoc.
   *
   * <p>Because it needs {@code Drive}, this can't run concurrently with a path-following command
   * the way {@link #deployIntake} can (both would fight over the chassis, and PathPlanner would
   * just cancel whichever started second). Sequence it AFTER the path that drives to the shooting
   * spot finishes -- either as its own step in a {@code .auto}'s command list, or an event marker
   * placed at the very end of the path (position 1.0), not mid-path.
   */
  public static Command shoot(
      Drive drive, Shooter shooter, Carriage carriage, Rollers rollers, Indexer indexer, Intake intake) {
    return ShootCommands.autoShoot(
        drive, shooter, carriage, rollers, indexer, intake, () -> 0.0, () -> 0.0, true);
  }

  /**
   * Pre-spin the flywheel + adjust the hood to the current distance-to-goal setpoint (same {@link
   * ShooterProfile} lookup {@link #shoot} uses), WITHOUT feeding -- meant to be dropped on a path
   * event marker well before the robot actually arrives at the shot, so the flywheel/hood are
   * already close to target once {@link #shoot} takes over. {@code shoot} also requires {@code
   * Shooter}, so scheduling it automatically cancels this one and hands off cleanly; no explicit
   * end condition needed between them.
   *
   * <p>Never finishes on its own ({@code isFinished()} is always false) -- drop it on a mid-path
   * event marker like {@link #deployIntake}, NOT as a step in a sequential {@code .auto} command
   * list (a sequence would block forever waiting for it to finish; it's only meant to be superseded
   * by a later Shoot marker, or cancelled when auto/teleop ends). Doesn't require {@code Drive} (it
   * only reads the pose for distance), so it's safe to run alongside a path-following command.
   */
  public static Command prespin(Drive drive, Shooter shooter) {
    return Commands.run(
            () -> {
              double distance = FieldConstants.distanceToGoal(drive.getPose().getTranslation());
              double targetSpeed =
                  USE_DISTANCE_PROFILE
                      ? ShooterProfile.speedForDistance(distance)
                      : BENCH_SHOOTER_RPS.get();
              double targetHood =
                  MathUtil.clamp(
                      USE_DISTANCE_PROFILE
                          ? ShooterProfile.hoodRotationsForDistance(distance)
                          : TARGET_HOOD_ROTATIONS.get(),
                      HOOD_MIN_ROTATIONS,
                      HOOD_MAX_ROTATIONS);

              shooter.setShooterMotorVelocity(targetSpeed);
              shooter.setHoodPosition(targetHood);
            },
            shooter)
        .finallyDo(
            () -> {
              shooter.stopShooter();
              shooter.setHoodPosition(HOOD_REST_ROTATIONS.get());
            });
  }

  /**
   * Deploy the carriage out and start the intake spinning. Both {@link Carriage#deployCommand()}
   * and {@link Intake#runCommand()} are {@code runOnce} -- this fires once and finishes almost
   * immediately, so it's safe to drop on a path event marker without blocking a sequence: the
   * carriage's MotionMagic profile and the roller velocity loop keep running on the TalonFX on
   * their own afterward, independent of whether this command is still scheduled. Doesn't require
   * {@code Drive}, so it's safe to run concurrently with a path-following command.
   */
  public static Command deployIntake(Carriage carriage, Intake intake) {
    return Commands.parallel(carriage.deployCommand(), intake.runCommand());
  }
}
