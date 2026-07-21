// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.AutonomousShootConfig.AUTO_SHOOT_TIMEOUT_SEC;
import static frc.robot.Constants.CarriageConfig.CARRIAGE_DOWN_POSITION;
import static frc.robot.Constants.CarriageConfig.CARRIAGE_UP_POSITION;
import static frc.robot.Constants.FeedConfig.FEED_HOOD_ROTATIONS;
import static frc.robot.Constants.IndexerConfig.INDEXER_VELOCITY;
import static frc.robot.Constants.ManualShootConfig.MANUAL_CARRIAGE_FAST_RETRACT_ROT_PER_SEC;
import static frc.robot.Constants.ManualShootConfig.MANUAL_CARRIAGE_RETRACT_ROT_PER_SEC;
import static frc.robot.Constants.ManualShootConfig.MANUAL_CARRIAGE_RETRACT_SPLIT_POSITION;
import static frc.robot.Constants.ManualShootConfig.MANUAL_HOOD_ROTATIONS;
import static frc.robot.Constants.ManualShootConfig.MANUAL_SHOOTER_RPS;
import static frc.robot.Constants.RollersConfig.ROLLERS_VELOCITY;
import static frc.robot.Constants.ShootSequenceConfig.AIM_DRIVE_ENABLED;
import static frc.robot.Constants.ShootSequenceConfig.BENCH_SHOOTER_RPS;
import static frc.robot.Constants.ShootSequenceConfig.CARRIAGE_FAST_RETRACT_ROT_PER_SEC;
import static frc.robot.Constants.ShootSequenceConfig.CARRIAGE_RETRACT_ROT_PER_SEC;
import static frc.robot.Constants.ShootSequenceConfig.CARRIAGE_RETRACT_SPLIT_POSITION;
import static frc.robot.Constants.ShootSequenceConfig.READY_DEBOUNCE_SEC;
import static frc.robot.Constants.ShootSequenceConfig.REQUIRE_ALIGNED_TO_FEED;
import static frc.robot.Constants.ShootSequenceConfig.TARGET_HOOD_ROTATIONS;
import static frc.robot.Constants.ShootSequenceConfig.USE_DISTANCE_PROFILE;
import static frc.robot.Constants.ShooterConfig.HOOD_MAX_ROTATIONS;
import static frc.robot.Constants.ShooterConfig.HOOD_MIN_ROTATIONS;
import static frc.robot.Constants.ShooterConfig.HOOD_REST_ROTATIONS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.subsystems.carriage.Carriage;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.shooter.FeedProfile;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterProfile;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Factory for the auto-shoot sequence, built from the standard command decorators ({@code
 * Commands.run(...).beforeStarting(...).finallyDo(...)}) instead of a Command subclass.
 *
 * <p>Pre-spin the flywheel + set the hood to the distance-interpolated shot ({@link
 * ShooterProfile}), turn the chassis to face the goal with a profiled heading PID, then once at
 * speed + at hood (+ aligned, if required) LATCH the feed on: rollers and indexer run continuously
 * (even as fed balls dip the flywheel speed) while the carriage slowly retracts.
 *
 * <p>Two flavors, both built by the same {@link #autoShoot(Drive, Shooter, Carriage, Rollers,
 * Indexer, Intake, DoubleSupplier, DoubleSupplier, boolean)}: the teleop one (bound with {@code
 * whileTrue} to the right trigger) runs for as long as held and re-deploys the carriage on
 * release; the autonomous one ({@code forAutonomous = true}, see {@code AutoCommands.shoot})
 * self-terminates once the shot completes and leaves the carriage retracted for a later deploy
 * action.
 */
public final class ShootCommands {
  private static final double DT = 0.020;

  private ShootCommands() {}

  /** Teleop flavor (right trigger, {@code whileTrue}): runs until interrupted, re-deploys on end. */
  public static Command autoShoot(
      Drive drive,
      Shooter shooter,
      Carriage carriage,
      Rollers rollers,
      Indexer indexer,
      Intake intake,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier) {
    return autoShoot(drive, shooter, carriage, rollers, indexer, intake, xSupplier, ySupplier, false);
  }

  /**
   * @param forAutonomous when true: self-terminates once the shot finishes (bounded by {@code
   *     AUTO_SHOOT_TIMEOUT_SEC} so a bad shot can't stall the rest of autonomous) instead of
   *     running until externally interrupted, and does NOT re-deploy the carriage at the end --
   *     it stays retracted until a later {@code AutoCommands.deployIntake} extends it back out.
   */
  public static Command autoShoot(
      Drive drive,
      Shooter shooter,
      Carriage carriage,
      Rollers rollers,
      Indexer indexer,
      Intake intake,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      boolean forAutonomous) {

    // State that must survive across loops: the heading controller and the ramped carriage target.
    ProfiledPIDController headingController =
        new ProfiledPIDController(
            DriveConstants.AIM_KP,
            DriveConstants.AIM_KI,
            DriveConstants.AIM_KD,
            new TrapezoidProfile.Constraints(
                DriveConstants.AIM_MAX_VELOCITY_RAD_PER_SEC,
                DriveConstants.AIM_MAX_ACCEL_RAD_PER_SEC2));
    headingController.enableContinuousInput(-Math.PI, Math.PI);
    headingController.setTolerance(Math.toRadians(DriveConstants.AIM_TOLERANCE_DEG.get()));
    // Smooths the heading controller's omega output -- see the constant's doc for why this filters
    // the output instead of the (wraparound-prone) input heading.
    LinearFilter omegaFilter =
        LinearFilter.singlePoleIIR(DriveConstants.AIM_OMEGA_FILTER_TIME_CONSTANT_SEC, DT);

    // Boxed so the run() lambda can mutate it; reset in beforeStarting().
    double[] carriageTarget = {CARRIAGE_DOWN_POSITION.get()};
    // Which leg of the double-compress retract we're on -- see stepCarriageRetract's javadoc.
    int[] retractPhase = {0};
    // Latched true the first moment readyToShoot is true; from then on the feed runs continuously
    // until the command ends (and the timer paces the carriage retract).
    boolean[] feedStarted = {false};
    // The heading commanded at the instant the feed latches, held fixed from then on (see below).
    double[] lockedHeadingRad = {0.0};
    // Same freeze treatment for the hood: locked at the instant the feed latches.
    double[] lockedHoodRot = {0.0};
    Timer feedTimer = new Timer();
    // atSpeed && atHood (&& aligned) must hold continuously for READY_DEBOUNCE_SEC before it counts
    // -- see the constant's doc in Constants.ShootSequenceConfig.
    Debouncer readyDebouncer = new Debouncer(READY_DEBOUNCE_SEC.get(), DebounceType.kRising);

    Command command =
        Commands.run(
            () -> {
              // --- Distance-based setpoints (red/blue aware: goal() mirrors for the alliance) ---
              Translation2d goal = FieldConstants.goal();
              Translation2d robot = drive.getPose().getTranslation();
              double distance = FieldConstants.distanceToGoal(robot);
              // Auto-ranging: look the speed + hood angle up from the distance interpolation table.
              // Bench mode (USE_DISTANCE_PROFILE=false) falls back to a single fixed speed/angle.
              double targetSpeed =
                  USE_DISTANCE_PROFILE
                      ? ShooterProfile.speedForDistance(distance)
                      : BENCH_SHOOTER_RPS.get();
              // Same lock as the heading: keep tracking the distance-derived hood angle while still
              // acquiring, then freeze it the instant the feed latches so the hood stops moving
              // mid-shot (a moving pitch during the feed is exactly the kind of jitter/inconsistency
              // this is meant to avoid).
              double targetHood;
              if (!feedStarted[0]) {
                targetHood =
                    USE_DISTANCE_PROFILE
                        ? ShooterProfile.hoodRotationsForDistance(distance)
                        : TARGET_HOOD_ROTATIONS.get();
                // Clamp to the hood's travel so the "at position" gate matches what setHoodPosition()
                // actually commands (it clamps internally too).
                targetHood = MathUtil.clamp(targetHood, HOOD_MIN_ROTATIONS, HOOD_MAX_ROTATIONS);
                lockedHoodRot[0] = targetHood;
              } else {
                targetHood = lockedHoodRot[0];
              }

              // --- Pre-spin flywheel + adjust hood (always, while held) ---
              shooter.setShooterMotorVelocity(targetSpeed);
              shooter.setHoodPosition(targetHood);

              // --- Turn to face the goal; driver keeps field-relative translation ---
              // Aim the SHOOTER end at the goal: atan2 gives the bearing to the goal, then
              // AIM_HEADING_OFFSET_RAD (=pi) rotates it so the rear-facing shooter points there.
              // Only re-aim while still acquiring the target. The instant the feed latches (balls are
              // actually going out), FREEZE the heading — continuing to re-solve targetHeading off the
              // live pose every loop chased small pose-estimator noise and made the chassis visibly
              // jitter while shooting. Locking in the last solved heading holds the chassis still for
              // the rest of the shot; the driver can still translate, just not rotate.
              double targetHeading;
              double omega;
              if (!feedStarted[0]) {
                targetHeading =
                    Math.atan2(goal.getY() - robot.getY(), goal.getX() - robot.getX())
                        + DriveConstants.AIM_HEADING_OFFSET_RAD;
                omega =
                    omegaFilter.calculate(
                        headingController.calculate(drive.getRotation().getRadians(), targetHeading));
                lockedHeadingRad[0] = targetHeading;
              } else {
                targetHeading = lockedHeadingRad[0];
                omega = 0;
              }
              if (AIM_DRIVE_ENABLED) {
                double maxV = drive.getMaxLinearSpeedMetersPerSec();
                double vx = deadband(xSupplier) * maxV;
                double vy = deadband(ySupplier) * maxV;
                // Operator perspective for the STICK only (see FieldConstants.operatorForward) --
                // the heading PID above still aims off the true rotation, this just keeps "push
                // stick forward" meaning "away from your own alliance wall" while shooting on red.
                drive.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        vx, vy, omega, FieldConstants.operatorForward(drive.getRotation())));
              }

              // --- Readiness gate (alignment optional; see REQUIRE_ALIGNED_TO_FEED) ---
              // Re-applied every loop so AIM_TOLERANCE_DEG can be dialed in live without a redeploy.
              headingController.setTolerance(Math.toRadians(DriveConstants.AUTO_AIM_TOLERANCE_DEG.get()));
              // When the chassis isn't aiming (bench mode) we can't turn, so treat as aligned.
              // atSetpoint() (position error within tolerance), NOT atGoal(): atGoal() additionally
              // requires the goal State to exactly equal the setpoint State, but targetHeading is
              // recomputed from the live pose every loop above, so the goal shifts by a hair each
              // tick and can never hold still long enough for that exact equality to land -- atGoal()
              // flickered false almost permanently even while the chassis was genuinely on target.
              boolean aligned = !AIM_DRIVE_ENABLED || headingController.atSetpoint();
              boolean atSpeed = shooter.atVelocity(targetSpeed);
              boolean atHood = shooter.atHoodPosition(targetHood);
              boolean rawReady = (!REQUIRE_ALIGNED_TO_FEED || aligned) && atSpeed && atHood;
              readyDebouncer.setDebounceTime(READY_DEBOUNCE_SEC.get());
              boolean readyToShoot = readyDebouncer.calculate(rawReady);
              SmartDashboard.putBoolean("AutoShoot/Aligned", aligned);
              SmartDashboard.putBoolean("AutoShoot/AtSpeed", atSpeed);
              SmartDashboard.putBoolean("AutoShoot/AtHood", atHood);
              SmartDashboard.putBoolean("AutoShoot/ReadyToShoot", readyToShoot);

              // Step 1: the first moment everything is ready, LATCH the feed on. It must NOT
              // re-check readiness afterwards: every ball fed dips the flywheel below tolerance,
              // and gating on readyToShoot each loop chopped the feed to 0 and back — the sawtooth
              // indexer velocity seen on the real robot. Once latched, feed until the command ends.
              if (readyToShoot && !feedStarted[0]) {
                feedStarted[0] = true;
                feedTimer.restart();
              }
              if (feedStarted[0]) {
                rollers.setRollersMotorVelocity(ROLLERS_VELOCITY.get());
                indexer.setIndexerMotorVelocity(INDEXER_VELOCITY.get());
                intake.spin(); // run the intake alongside the feed
              } else {
                rollers.setRollersMotorVelocity(0);
                indexer.setIndexerMotorVelocity(0);
                intake.stop();
              }
              SmartDashboard.putBoolean("AutoShoot/FeedCommanded", feedStarted[0]);

              // Step 2: the instant the feed latches, start the "double compress" retract -- no more
              // holding at full deploy first (see stepCarriageRetract's javadoc).
              boolean retracting = feedStarted[0];
              if (retracting) {
                carriageTarget[0] =
                    stepCarriageRetract(
                        carriageTarget[0],
                        retractPhase,
                        CARRIAGE_FAST_RETRACT_ROT_PER_SEC.get(),
                        CARRIAGE_RETRACT_ROT_PER_SEC.get(),
                        CARRIAGE_RETRACT_SPLIT_POSITION.get());
                carriage.setDeployPosition(carriageTarget[0]);
              }

              Logger.recordOutput("AutoShoot/DistanceM", distance);
              Logger.recordOutput("AutoShoot/TargetSpeedRPS", targetSpeed);
              Logger.recordOutput("AutoShoot/TargetHoodRot", targetHood);
              Logger.recordOutput("AutoShoot/GoalX", goal.getX());
              Logger.recordOutput("AutoShoot/GoalY", goal.getY());
              Logger.recordOutput("AutoShoot/IsRedAlliance", FieldConstants.isRedAlliance());
              Logger.recordOutput("AutoShoot/TargetHeadingRad", targetHeading);
              Logger.recordOutput(
                  "AutoShoot/HeadingErrorRad",
                  headingController.getSetpoint().position - drive.getRotation().getRadians());
              Logger.recordOutput("AutoShoot/Aligned", aligned);
              Logger.recordOutput("AutoShoot/AtSpeed", atSpeed);
              Logger.recordOutput("AutoShoot/AtHood", atHood);
              Logger.recordOutput("AutoShoot/ReadyToShoot", readyToShoot);
              Logger.recordOutput("AutoShoot/FeedStarted", feedStarted[0]);
              Logger.recordOutput("AutoShoot/FeedElapsedSec", feedTimer.get());
              Logger.recordOutput("AutoShoot/Retracting", retracting);
              Logger.recordOutput("AutoShoot/CarriageTargetRot", carriageTarget[0]);
            },
            drive,
            shooter,
            carriage,
            rollers,
            indexer,
            intake)
        // initialize(): ease the heading profile in from the current heading and extend the carriage.
        .beforeStarting(
            () -> {
              headingController.reset(drive.getRotation().getRadians());
              omegaFilter.reset();
              carriageTarget[0] = CARRIAGE_DOWN_POSITION.get();
              retractPhase[0] = 0;
              carriage.setDeployPosition(carriageTarget[0]);
              feedStarted[0] = false;
              lockedHeadingRad[0] = drive.getRotation().getRadians();
              lockedHoodRot[0] = HOOD_REST_ROTATIONS.get();
              feedTimer.stop();
              feedTimer.reset();
              readyDebouncer.calculate(false); // re-arm: must hold ready continuously from scratch
            });

    // Autonomous: self-terminate once the carriage setpoint ramps down to fully retracted (the
    // shot is done), bounded by a hard timeout so a shot that never reaches speed/hood can't stall
    // the rest of autonomous. Teleop leaves this off and just runs until the button is released.
    if (forAutonomous) {
      command =
          command
              .until(() -> feedStarted[0] && carriageTarget[0] <= CARRIAGE_UP_POSITION.get())
              .withTimeout(AUTO_SHOOT_TIMEOUT_SEC.get());
    }

    // end(): stop everything, return the hood to rest. Teleop also re-deploys the carriage (so the
    // driver can immediately intake again); autonomous leaves it retracted for a later deploy action.
    return command.finallyDo(
        () -> {
          shooter.stopShooter(); // coast down on its own inertia, don't brake
          shooter.setHoodPosition(HOOD_REST_ROTATIONS.get());
          rollers.setRollersMotorVelocity(0);
          indexer.setIndexerMotorVelocity(0);
          intake.stop();
          if (!forAutonomous) {
            carriage.setDeployPosition(CARRIAGE_DOWN_POSITION.get()); // return to deployed/down
          }
        });
  }

  /**
   * Corner-feed shot, ported from 25-26-swerve's {@code createCornerFeedCommand}. Bound to the
   * right trigger instead of {@link #autoShoot} once the robot is physically past the goal (see
   * {@code RobotContainer.pastFeedZone()}): rather than aiming at the goal, aims at whichever feed
   * corner ({@link FieldConstants#nearestFeedCorner}) is closer, and looks up flywheel speed from
   * {@link FeedProfile} instead of {@link ShooterProfile}
   * (a lobbed feed needs a different speed curve than a shot at the goal, and doesn't need the
   * PID/bang-bang split — see FeedProfile's javadoc). The hood does NOT track distance here — it's
   * held at the single fixed {@code Constants.FeedConfig.FEED_HOOD_ROTATIONS} for every feed shot,
   * only the flywheel speed varies with range (mirrors how FRC2910/FRC6328 do their cross-field
   * pass). Same aim-then-latch-feed-then-slow-retract choreography as {@link #autoShoot}, always
   * requiring the chassis to be aligned before it feeds (unlike autoShoot there's no bench-mode
   * toggle here).
   */
  public static Command feed(
      Drive drive,
      Shooter shooter,
      Carriage carriage,
      Rollers rollers,
      Indexer indexer,
      Intake intake,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier) {

    ProfiledPIDController headingController =
        new ProfiledPIDController(
            DriveConstants.AIM_KP,
            DriveConstants.AIM_KI,
            DriveConstants.AIM_KD,
            new TrapezoidProfile.Constraints(
                DriveConstants.AIM_MAX_VELOCITY_RAD_PER_SEC,
                DriveConstants.AIM_MAX_ACCEL_RAD_PER_SEC2));
    headingController.enableContinuousInput(-Math.PI, Math.PI);
    headingController.setTolerance(Math.toRadians(DriveConstants.AIM_TOLERANCE_DEG.get()));
    LinearFilter omegaFilter =
        LinearFilter.singlePoleIIR(DriveConstants.AIM_OMEGA_FILTER_TIME_CONSTANT_SEC, DT);

    // Boxed so the run() lambda can mutate it; reset in beforeStarting(). Same roles as autoShoot.
    double[] carriageTarget = {CARRIAGE_DOWN_POSITION.get()};
    int[] retractPhase = {0};
    boolean[] feedStarted = {false};
    double[] lockedHeadingRad = {0.0};
    Timer feedTimer = new Timer();
    Debouncer readyDebouncer = new Debouncer(READY_DEBOUNCE_SEC.get(), DebounceType.kRising);

    return Commands.run(
            () -> {
              // --- Distance-based speed to the nearer feed corner (red/blue aware); hood is fixed ---
              Translation2d robot = drive.getPose().getTranslation();
              Translation2d target = FieldConstants.nearestFeedCorner(robot);
              double distance = robot.getDistance(target);
              double targetSpeed = FeedProfile.speedForDistance(distance);

              // Hood is locked at one angle for every feed shot (see FeedConfig javadoc) -- no
              // distance tracking/freezing needed, unlike autoShoot's distance-varying hood.
              double targetHood =
                  MathUtil.clamp(FEED_HOOD_ROTATIONS.get(), HOOD_MIN_ROTATIONS, HOOD_MAX_ROTATIONS);

              shooter.setShooterMotorVelocity(targetSpeed);
              shooter.setHoodPosition(targetHood);

              // --- Turn to face the target corner; driver keeps field-relative translation ---
              double targetHeading;
              double omega;
              if (!feedStarted[0]) {
                targetHeading =
                    Math.atan2(target.getY() - robot.getY(), target.getX() - robot.getX())
                        + DriveConstants.AIM_HEADING_OFFSET_RAD;
                omega =
                    omegaFilter.calculate(
                        headingController.calculate(drive.getRotation().getRadians(), targetHeading));
                lockedHeadingRad[0] = targetHeading;
              } else {
                targetHeading = lockedHeadingRad[0];
                omega = 0;
              }
              double maxV = drive.getMaxLinearSpeedMetersPerSec();
              double vx = deadband(xSupplier) * maxV;
              double vy = deadband(ySupplier) * maxV;
              // Operator perspective for the STICK only (see FieldConstants.operatorForward) -- the
              // heading PID above still aims off the true rotation.
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      vx, vy, omega, FieldConstants.operatorForward(drive.getRotation())));

              // --- Readiness gate: always require alignment (no bench-mode toggle for feed) ---
              headingController.setTolerance(Math.toRadians(DriveConstants.AIM_TOLERANCE_DEG.get()));
              // atSetpoint(), not atGoal() -- see the doc note in autoShoot() above; same issue here
              // since targetHeading is likewise recomputed from the live pose every loop.
              boolean aligned = headingController.atSetpoint();
              boolean atSpeed = shooter.atVelocity(targetSpeed);
              boolean atHood = shooter.atHoodPosition(targetHood);
              boolean rawReady = aligned && atSpeed && atHood;
              readyDebouncer.setDebounceTime(READY_DEBOUNCE_SEC.get());
              boolean readyToShoot = readyDebouncer.calculate(rawReady);

              if (readyToShoot && !feedStarted[0]) {
                feedStarted[0] = true;
                feedTimer.restart();
              }
              if (feedStarted[0]) {
                rollers.setRollersMotorVelocity(ROLLERS_VELOCITY.get());
                indexer.setIndexerMotorVelocity(INDEXER_VELOCITY.get());
                intake.spin();
              } else {
                rollers.setRollersMotorVelocity(0);
                indexer.setIndexerMotorVelocity(0);
                intake.stop();
              }

              boolean retracting = feedStarted[0];
              if (retracting) {
                carriageTarget[0] =
                    stepCarriageRetract(
                        carriageTarget[0],
                        retractPhase,
                        CARRIAGE_FAST_RETRACT_ROT_PER_SEC.get(),
                        CARRIAGE_RETRACT_ROT_PER_SEC.get(),
                        CARRIAGE_RETRACT_SPLIT_POSITION.get());
                carriage.setDeployPosition(carriageTarget[0]);
              }

              Logger.recordOutput("Feed/DistanceM", distance);
              Logger.recordOutput("Feed/TargetX", target.getX());
              Logger.recordOutput("Feed/TargetY", target.getY());
              Logger.recordOutput("Feed/TargetSpeedRPS", targetSpeed);
              Logger.recordOutput("Feed/TargetHoodRot", targetHood);
              Logger.recordOutput("Feed/Aligned", aligned);
              Logger.recordOutput("Feed/AtSpeed", atSpeed);
              Logger.recordOutput("Feed/AtHood", atHood);
              Logger.recordOutput("Feed/ReadyToShoot", readyToShoot);
              Logger.recordOutput("Feed/FeedStarted", feedStarted[0]);
              Logger.recordOutput("Feed/Retracting", retracting);
              Logger.recordOutput("Feed/CarriageTargetRot", carriageTarget[0]);
            },
            drive,
            shooter,
            carriage,
            rollers,
            indexer,
            intake)
        .beforeStarting(
            () -> {
              headingController.reset(drive.getRotation().getRadians());
              omegaFilter.reset();
              carriageTarget[0] = CARRIAGE_DOWN_POSITION.get();
              retractPhase[0] = 0;
              carriage.setDeployPosition(carriageTarget[0]);
              feedStarted[0] = false;
              lockedHeadingRad[0] = drive.getRotation().getRadians();
              feedTimer.stop();
              feedTimer.reset();
              readyDebouncer.calculate(false); // re-arm: must hold ready continuously from scratch
            })
        .finallyDo(
            () -> {
              shooter.stopShooter(); // coast down on its own inertia, don't brake
              shooter.setHoodPosition(HOOD_REST_ROTATIONS.get());
              rollers.setRollersMotorVelocity(0);
              indexer.setIndexerMotorVelocity(0);
              intake.stop();
              carriage.setDeployPosition(CARRIAGE_DOWN_POSITION.get()); // return to deployed/down
            });
  }

  /**
   * Manual (no-vision) shot — bound to the left trigger. Runs the same feed choreography as {@link
   * #autoShoot} but with FIXED, live-tunable setpoints instead of the distance profile, and it never
   * touches the chassis (the driver keeps full manual control while shooting). Every number lives
   * under {@code /Tuning/ManualShoot/*} in AdvantageScope.
   *
   * <p>While held: spin the flywheel to {@code MANUAL_SHOOTER_RPS} and set the hood to {@code
   * MANUAL_HOOD_ROTATIONS}; once at speed + at hood, LATCH the feed (rollers + indexer) on and
   * immediately begin the "double compress" retract (see {@link #stepCarriageRetract}). On release
   * everything stops, the hood returns to rest, and the carriage returns to the deployed/down
   * position.
   */
  public static Command manualShoot(
      Drive drive, Shooter shooter, Carriage carriage, Rollers rollers, Indexer indexer, Intake intake) {

    // Touch these once now, outside the run() lambda below: Java only runs ManualShootConfig's
    // static field initializers -- which is what actually constructs each LoggedTunableNumber and
    // publishes its NT entry -- the first time one of its fields is read. All four are otherwise
    // only read inside the lambda, which doesn't execute until the left trigger is actually held,
    // so without this they silently wouldn't show up in AdvantageScope until the first manual shot.
    MANUAL_SHOOTER_RPS.get();
    MANUAL_HOOD_ROTATIONS.get();
    MANUAL_CARRIAGE_FAST_RETRACT_ROT_PER_SEC.get();
    MANUAL_CARRIAGE_RETRACT_ROT_PER_SEC.get();
    MANUAL_CARRIAGE_RETRACT_SPLIT_POSITION.get();

    // Boxed so the run() lambda can mutate it; reset in beforeStarting().
    double[] carriageTarget = {CARRIAGE_DOWN_POSITION.get()};
    int[] retractPhase = {0};
    boolean[] feedStarted = {false};
    Timer feedTimer = new Timer();
    Debouncer readyDebouncer = new Debouncer(READY_DEBOUNCE_SEC.get(), DebounceType.kRising);

    return Commands.run(
            () -> {
              // --- Fixed setpoints (no vision) ---
              double targetSpeed = MANUAL_SHOOTER_RPS.get();
              double targetHood =
                  MathUtil.clamp(
                      MANUAL_HOOD_ROTATIONS.get(), HOOD_MIN_ROTATIONS, HOOD_MAX_ROTATIONS);

              // --- Debug range (drive is NOT a requirement, so the driver keeps control) ---
              // Distance to the hub center (red/blue aware), for calibrating the manual speed/hood
              // against a known range while building the ShooterProfile table.
              double distance =
                  FieldConstants.distanceToGoal(drive.getPose().getTranslation());

              // --- Pre-spin flywheel + adjust hood (always, while held) ---
              shooter.setShooterMotorVelocity(targetSpeed);
              shooter.setHoodPosition(targetHood);

              // --- Readiness gate: no alignment, just at-speed + at-hood ---
              boolean atSpeed = shooter.atVelocity(targetSpeed);
              boolean atHood = shooter.atHoodPosition(targetHood);
              boolean rawReady = atSpeed && atHood;
              readyDebouncer.setDebounceTime(READY_DEBOUNCE_SEC.get());
              boolean readyToShoot = readyDebouncer.calculate(rawReady);

              // Latch the feed on the first ready moment; don't re-gate afterwards (fed balls dip the
              // flywheel below tolerance and would chop the feed into a sawtooth).
              if (readyToShoot && !feedStarted[0]) {
                feedStarted[0] = true;
                feedTimer.restart();
              }
              if (feedStarted[0]) {
                rollers.setRollersMotorVelocity(ROLLERS_VELOCITY.get());
                indexer.setIndexerMotorVelocity(INDEXER_VELOCITY.get());
                intake.spin(); // run the intake alongside the feed
              } else {
                rollers.setRollersMotorVelocity(0);
                indexer.setIndexerMotorVelocity(0);
                intake.stop();
              }

              // The instant the feed latches, start the "double compress" retract.
              boolean retracting = feedStarted[0];
              if (retracting) {
                carriageTarget[0] =
                    stepCarriageRetract(
                        carriageTarget[0],
                        retractPhase,
                        MANUAL_CARRIAGE_FAST_RETRACT_ROT_PER_SEC.get(),
                        MANUAL_CARRIAGE_RETRACT_ROT_PER_SEC.get(),
                        MANUAL_CARRIAGE_RETRACT_SPLIT_POSITION.get());
                carriage.setDeployPosition(carriageTarget[0]);
              }

              // Live debug readout for Elastic while the left trigger is held: current (measured)
              // flywheel speed + hood angle alongside the range to the hub, so you can read off a
              // {distance, speed, hood} row for the ShooterProfile table.
              SmartDashboard.putNumber("Manual/Distance to Hub (m)", distance);
              SmartDashboard.putNumber("Manual/Shooter Speed (rps)", shooter.getVelocityRPS());
              SmartDashboard.putNumber("Manual/Hood Angle (rot)", shooter.getHoodPositionRotations());

              Logger.recordOutput("ManualShoot/DistanceM", distance);
              Logger.recordOutput("ManualShoot/TargetSpeedRPS", targetSpeed);
              Logger.recordOutput("ManualShoot/TargetHoodRot", targetHood);
              Logger.recordOutput("ManualShoot/AtSpeed", atSpeed);
              Logger.recordOutput("ManualShoot/AtHood", atHood);
              Logger.recordOutput("ManualShoot/ReadyToShoot", readyToShoot);
              Logger.recordOutput("ManualShoot/FeedStarted", feedStarted[0]);
              Logger.recordOutput("ManualShoot/FeedElapsedSec", feedTimer.get());
              Logger.recordOutput("ManualShoot/Retracting", retracting);
              Logger.recordOutput("ManualShoot/CarriageTargetRot", carriageTarget[0]);
            },
            shooter,
            carriage,
            rollers,
            indexer,
            intake)
        .beforeStarting(
            () -> {
              carriageTarget[0] = CARRIAGE_DOWN_POSITION.get();
              retractPhase[0] = 0;
              carriage.setDeployPosition(carriageTarget[0]);
              feedStarted[0] = false;
              feedTimer.stop();
              feedTimer.reset();
              readyDebouncer.calculate(false); // re-arm: must hold ready continuously from scratch
            })
        .finallyDo(
            () -> {
              shooter.stopShooter(); // coast down on its own inertia, don't brake
              shooter.setHoodPosition(HOOD_REST_ROTATIONS.get());
              rollers.setRollersMotorVelocity(0);
              indexer.setIndexerMotorVelocity(0);
              intake.stop();
              carriage.setDeployPosition(CARRIAGE_DOWN_POSITION.get()); // back to the deployed/down position
            });
  }

  /** Deadbanded, sign-corrected stick value in [-1, 1] (matches DriveCommands). */
  private static double deadband(DoubleSupplier stick) {
    return -MathUtil.applyDeadband(stick.getAsDouble(), DriveConstants.JOYSTICK_DEADBAND);
  }

  /**
   * "Double compress" carriage retract, one step of it (called once per loop): pull in (fast) to
   * {@code splitPosition}, push back out (fast) to fully deployed, then pull all the way in to
   * fully retracted (slow, so it eases in instead of slamming). Three phases tracked via {@code
   * phase[0]} (0 = pulling to the split point, 1 = pushing back out, 2 = final pull to fully
   * retracted) since the target is no longer monotonic -- position alone can't tell phase 1 (still
   * short of fully deployed) from phase 2 passing back through the same split point. Replaces the
   * old "hold deployed for a fixed delay, then retract at one constant rate" choreography -- this
   * starts the instant the feed latches, no delay first.
   */
  private static double stepCarriageRetract(
      double currentTarget,
      int[] phase,
      double fastRotPerSec,
      double slowRotPerSec,
      double splitPosition) {
    double down = CARRIAGE_DOWN_POSITION.get();
    double up = CARRIAGE_UP_POSITION.get();

    switch (phase[0]) {
      case 0 -> {
        currentTarget = MathUtil.clamp(currentTarget - fastRotPerSec * DT, splitPosition, down);
        if (currentTarget <= splitPosition) {
          phase[0] = 1;
        }
      }
      case 1 -> {
        currentTarget = MathUtil.clamp(currentTarget + fastRotPerSec * DT, splitPosition, down);
        if (currentTarget >= down) {
          phase[0] = 2;
        }
      }
      default -> currentTarget = MathUtil.clamp(currentTarget - slowRotPerSec * DT, up, down);
    }
    return currentTarget;
  }
}
