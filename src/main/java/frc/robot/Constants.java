// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.util.LoggedTunableNumber;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.
 *
 * <p>Behavioral constants (PID gains, velocities, setpoints, timings, tolerances, current limits)
 * are declared as {@link LoggedTunableNumber} so they can be edited live from this file's dashboard
 * entries under "/Tuning/*" in AdvantageScope/NetworkTables. Structural constants (CAN IDs, gear /
 * sensor ratios, calibration offsets, sim-only physics) stay plain {@code static final} values —
 * they're only ever read once at hardware/sim construction, so making them "live" would do nothing.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
  }

  // AdvantageKit runtime mode. REAL on a roboRIO; SIM in the physics sim; REPLAY when re-running a log.
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode =
      edu.wpi.first.wpilibj.RobotBase.isReal() ? Mode.REAL : simMode;

  // When true, PID/feedforward gains and setpoints are exposed as live-editable tunable numbers
  // (published under "/Tuning/*" in NetworkTables) so they can be tuned from AdvantageScope.
  // Turn OFF for competition so no tuning entries are created.
  public static final boolean TUNING_MODE = true;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,
    /** Running a physics simulator. */
    SIM,
    /** Replaying from a log file. */
    REPLAY
  }

  // CAN IDs 1-8 reserved for swerve drivetrain

  public static final class ShooterConfig {
    public static final int SHOOTER_LEFT_UP_MOTOR_ID    = 9;
    public static final int SHOOTER_LEFT_DOWN_MOTOR_ID  = 10;
    public static final int SHOOTER_RIGHT_UP_MOTOR_ID   = 11;
    public static final int SHOOTER_RIGHT_DOWN_MOTOR_ID = 12;

    // Reverse-spin speed for the outtake (Y button) — the only remaining caller now that the old
    // Y test-spin button is gone. rotor rps.
    public static final LoggedTunableNumber SHOOTER_OUTTAKE_VELOCITY =
        new LoggedTunableNumber("Shooter/OuttakeVelocity", 30.0);

    // ---- Velocity loop (VelocityTorqueCurrentFOC, amps/rotor-rps); sim physics in SimConfig ----
    public static final LoggedTunableNumber SHOOTER_KS =
        new LoggedTunableNumber("Shooter/kS", 0.0); // A  (frictionless sim; tune kS/kV on real hw)
    public static final LoggedTunableNumber SHOOTER_KV =
        new LoggedTunableNumber("Shooter/kV", 0.0); // A / rotor-rps
    public static final LoggedTunableNumber SHOOTER_KP =
        new LoggedTunableNumber("Shooter/kP", 5.0); // A / rotor-rps of error
    public static final LoggedTunableNumber SHOOTER_KI =
        new LoggedTunableNumber("Shooter/kI", 0.0);
    public static final LoggedTunableNumber SHOOTER_KD =
        new LoggedTunableNumber("Shooter/kD", 0.0);
    public static final LoggedTunableNumber SHOOTER_TORQUE_CURRENT_LIMIT =
        new LoggedTunableNumber("Shooter/TorqueCurrentLimit", 80.0); // A

    // "At speed" gate for the auto-shoot sequence (how close the flywheel must be before feeding).
    public static final LoggedTunableNumber SHOOTER_VELOCITY_TOLERANCE_RPS =
        new LoggedTunableNumber("Shooter/VelocityToleranceRPS", 2.0);

    // ---- Hood pivot: one TalonFX + one fused CANcoder, MotionMagicVoltage position control ----
    // Lives inside the Shooter subsystem (hood + flywheel are one physical mechanism). The CANcoder
    // sits on the hood axis (SensorToMechanismRatio = 1), so the hood position is in MECHANISM
    // ROTATIONS; the whole control path (setpoints, soft limits, ShooterProfile) uses rotations.
    public static final int SHOOTER_HOOD_MOTOR_ID    = 20;
    public static final int SHOOTER_HOOD_CANCODER_ID = 0;

    public static final double HOOD_GEAR_RATIO = 50.0;  // rotor rotations per hood rotation (RotorToSensor)
    public static final double HOOD_MIN_ROTATIONS = 0.0;  // hard low stop (reverse soft limit)
    public static final double HOOD_MAX_ROTATIONS = 1.7;  // hard high stop (forward soft limit)
    public static final LoggedTunableNumber HOOD_REST_ROTATIONS =
        new LoggedTunableNumber("Shooter/Hood/RestRotations", HOOD_MIN_ROTATIONS); // idle/stow position
    public static final LoggedTunableNumber HOOD_TOLERANCE_ROTATIONS =
        new LoggedTunableNumber("Shooter/Hood/ToleranceRotations", 0.5); // "at position" gate
    public static final double HOOD_CANCODER_OFFSET_ROT = -0.2026; // magnet offset (rotations), tune on hw

    // MotionMagicVoltage gains (VOLTS; mechanism reads/commands in CANcoder rotations of the hood).
    public static final LoggedTunableNumber HOOD_KS =
        new LoggedTunableNumber("Shooter/Hood/kS", 0.0); // V (static friction; tune on hw)
    public static final LoggedTunableNumber HOOD_KV =
        new LoggedTunableNumber("Shooter/Hood/kV", 0.0); // V / (rot/s)
    public static final LoggedTunableNumber HOOD_KA =
        new LoggedTunableNumber("Shooter/Hood/kA", 0.0); // V / (rot/s^2)
    public static final LoggedTunableNumber HOOD_KP =
        new LoggedTunableNumber("Shooter/Hood/kP", 5.0); // V / rot of error
    public static final LoggedTunableNumber HOOD_KI =
        new LoggedTunableNumber("Shooter/Hood/kI", 0.0);
    public static final LoggedTunableNumber HOOD_KD =
        new LoggedTunableNumber("Shooter/Hood/kD", 0.0); // V / (rot/s)
    public static final LoggedTunableNumber HOOD_MM_CRUISE_RPS =
        new LoggedTunableNumber("Shooter/Hood/CruiseVelocity", 500.0); // hood rot/s
    public static final LoggedTunableNumber HOOD_MM_ACCEL_RPS2 =
        new LoggedTunableNumber("Shooter/Hood/Acceleration", 1500.0); // hood rot/s^2
  }

  /** Tunables for the full press-trigger auto-shoot sequence ({@code ShootCommands.autoShoot}). */
  public static final class ShootSequenceConfig {
    // ---- Auto-aim / auto-ranging master switches ----------------------------------------------
    // The auto-shoot sequence has two automatic behaviors that a robot on a bench/stand can't do:
    //   USE_DISTANCE_PROFILE  true -> flywheel speed + hood angle come from ShooterProfile (the
    //                                 distance interpolation table). false -> use the fixed BENCH_*
    //                                 setpoints below, so you can spin at one known speed/angle.
    //   AIM_DRIVE_ENABLED     true -> the chassis rotates to face the goal (red/blue aware). false ->
    //                                 no chassis rotation is commanded, and the "aligned" feed gate is
    //                                 forced true (a stand can't turn, so alignment would never finish).
    // Keep BOTH true on the field / in sim; flip BOTH false for bench work on a stand.
    public static final boolean USE_DISTANCE_PROFILE = true;
    public static final boolean AIM_DRIVE_ENABLED = true;

    // Fixed setpoints used when USE_DISTANCE_PROFILE is false (bench mode).
    public static final LoggedTunableNumber BENCH_SHOOTER_RPS =
        new LoggedTunableNumber("ShootSequence/BenchShooterRPS", 50.0); // flywheel rotor rps
    public static final LoggedTunableNumber TARGET_HOOD_ROTATIONS =
        new LoggedTunableNumber("ShootSequence/TargetHoodRotations", 1.0); // bench hood mechanism rot

    // "Double compress" retract: starts the instant the feed latches (no more holding at full
    // deploy first) -- pull in (fast) to CARRIAGE_RETRACT_SPLIT_POSITION, push back out (fast) to
    // DOWN, then pull all the way to UP at this slower rate so it eases in instead of slamming
    // (deploy ROTOR ROTATIONS/sec).
    public static final LoggedTunableNumber CARRIAGE_FAST_RETRACT_ROT_PER_SEC =
        new LoggedTunableNumber("ShootSequence/CarriageFastRetractRotPerSec", 20.0);
    public static final LoggedTunableNumber CARRIAGE_RETRACT_ROT_PER_SEC =
        new LoggedTunableNumber("ShootSequence/CarriageRetractRotPerSec", 15.0);
    // Carriage position (rotor rotations, same units/frame as CARRIAGE_DOWN_POSITION /
    // CARRIAGE_UP_POSITION) the first "pull in" leg targets before pushing back out to DOWN and
    // then pulling all the way to UP. Default = the old 0.6 split fraction's equivalent point.
    public static final LoggedTunableNumber CARRIAGE_RETRACT_SPLIT_POSITION =
        new LoggedTunableNumber("ShootSequence/CarriageRetractSplitPosition", 5);

    // When TRUE the feed (rollers + indexer) only runs once the chassis has finished aiming at the
    // goal. Set FALSE for bench testing: on a stand the robot can't rotate, so alignment never
    // completes and the feed would never fire — with this false it feeds as soon as the flywheel and
    // hood are at their setpoints. Keep TRUE on the field so it only shoots while on target.
    public static final boolean REQUIRE_ALIGNED_TO_FEED = true;

    // "At speed" and "at hood" must hold true continuously for this long before the feed latches on
    // (mirrors Team 254's on-target sample-count technique) -- a single noisy loop tick briefly
    // brushing tolerance shouldn't be enough to fire. Shared by autoShoot, feed, and manualShoot.
    public static final LoggedTunableNumber READY_DEBOUNCE_SEC =
        new LoggedTunableNumber("ShootSequence/ReadyDebounceSec", 0.15);
  }

  /**
   * The "feed" shot ({@code ShootCommands.feed}, ported from 25-26-swerve): lobs to the nearer
   * feed corner instead of the goal, using {@link frc.robot.subsystems.shooter.FeedProfile}'s
   * distance table for speed instead of {@link frc.robot.subsystems.shooter.ShooterProfile}'s.
   * RobotContainer picks feed vs. the normal goal shot on the right trigger based on this zone.
   *
   * <p>Unlike the goal shot, the hood does NOT track distance for a feed -- it's held at a single
   * fixed {@link #FEED_HOOD_ROTATIONS}, only the flywheel speed varies with range. This mirrors how
   * FRC2910 and FRC6328's 2026 codebases both implement a cross-field pass: pin the hood near its
   * max (lob) angle and let speed alone cover the distance, instead of re-shaping the trajectory per
   * shot the way a precise hub shot does.
   */
  public static final class FeedConfig {
    // Locked hood angle for every feed shot, regardless of distance (mechanism rotations, within
    // [HOOD_MIN_ROTATIONS, HOOD_MAX_ROTATIONS]). Near max so the lob clears traffic on the way over
    // midfield; kept a bit below the hard stop (1.7) rather than pinned exactly to it.
    public static final LoggedTunableNumber FEED_HOOD_ROTATIONS =
        new LoggedTunableNumber("Feed/HoodRotations", 1.2);
  }

  /**
   * Manual shot (left trigger): no vision, no auto-aim, no distance profile — just the shooting
   * action at these FIXED setpoints. Everything is a {@link LoggedTunableNumber} so it can be edited
   * live in AdvantageScope under "/Tuning/ManualShoot/*". See {@code ShootCommands.manualShoot}.
   */
  public static final class ManualShootConfig {
    public static final LoggedTunableNumber MANUAL_SHOOTER_RPS =
        new LoggedTunableNumber("ManualShoot/ShooterRPS", 50.0); // flywheel rotor rps
    public static final LoggedTunableNumber MANUAL_HOOD_ROTATIONS =
        new LoggedTunableNumber("ManualShoot/HoodRotations", 1.0); // hood mechanism rotations
    // "Double compress" retract, same idea as ShootSequenceConfig's -- starts the instant the feed
    // latches: pull in (fast) to MANUAL_CARRIAGE_RETRACT_SPLIT_POSITION, push back out (fast) to
    // DOWN, then pull all the way to UP (slow) (deploy rotor rotations per second).
    public static final LoggedTunableNumber MANUAL_CARRIAGE_FAST_RETRACT_ROT_PER_SEC =
        new LoggedTunableNumber("ManualShoot/CarriageFastRetractRotPerSec", 30.0);
    public static final LoggedTunableNumber MANUAL_CARRIAGE_RETRACT_ROT_PER_SEC =
        new LoggedTunableNumber("ManualShoot/CarriageRetractRotPerSec", 20.0);
    // Carriage position (rotor rotations, same units/frame as CARRIAGE_DOWN_POSITION /
    // CARRIAGE_UP_POSITION). Default = the old 0.6 split fraction's equivalent point.
    public static final LoggedTunableNumber MANUAL_CARRIAGE_RETRACT_SPLIT_POSITION =
        new LoggedTunableNumber("ManualShoot/CarriageRetractSplitPosition", 7.2);
  }

  /**
   * The autonomous "shoot" action ({@code AutoCommands.shoot}) reuses {@code
   * ShootCommands.autoShoot} as-is -- same distance-interpolated {@link
   * frc.robot.subsystems.shooter.ShooterProfile} speed/hood and the same goal-facing aim, not a
   * fixed preset (setpoints live under {@link ShootSequenceConfig} / {@link ShooterConfig}, same
   * as the teleop version). This just adds the one thing autonomous needs on top: a hard bound on
   * how long that action is allowed to run before giving up and moving on.
   */
  public static final class AutonomousShootConfig {
    // Safety bound: if it never reaches speed/hood (goal out of range, hardware hiccup), don't let
    // this action stall the rest of autonomous -- end and move on regardless.
    public static final LoggedTunableNumber AUTO_SHOOT_TIMEOUT_SEC =
        new LoggedTunableNumber("AutonomousShoot/TimeoutSec", 3.0);
  }

  /** Intake rollers (the spinning wheels that pull a game piece in). Deploy slide is {@link CarriageConfig}. */
  public static final class IntakeConfig {
    public static final int INTAKE_LEFT_MOTOR_ID    = 13;
    public static final int INTAKE_RIGHT_MOTOR_ID   = 14;

    // ---- Setpoint (real-world units at the subsystem API boundary); used both directions — positive
    // to spin in, negated to spin out during outtake. ----
    public static final LoggedTunableNumber INTAKE_VELOCITY =
        new LoggedTunableNumber("Intake/Velocity", 80.0); // roller rps

    // ---- Roller velocity loop: VelocityTorqueCurrentFOC (gains in AMPS per rotor-rps) ----
    // Used by BOTH sim and real — one tuning. The sim plant is frictionless, so kS/kV are 0;
    // the real robot will need kS (and maybe kV) tuned on hardware to hold speed against friction.
    public static final LoggedTunableNumber INTAKE_KS =
        new LoggedTunableNumber("Intake/kS", 0.0); // A  (static friction)
    public static final LoggedTunableNumber INTAKE_KV =
        new LoggedTunableNumber("Intake/kV", 0.0); // A / (rotor rps)  (viscous)
    public static final LoggedTunableNumber INTAKE_KP =
        new LoggedTunableNumber("Intake/kP", 3.0); // A / (rotor rps of error)
    public static final LoggedTunableNumber INTAKE_KI =
        new LoggedTunableNumber("Intake/kI", 0.0);
    public static final LoggedTunableNumber INTAKE_KD =
        new LoggedTunableNumber("Intake/kD", 0.0);
    public static final LoggedTunableNumber INTAKE_TORQUE_CURRENT_LIMIT =
        new LoggedTunableNumber("Intake/TorqueCurrentLimit", 60.0); // A, peak |torque current|
  }

  /** Carriage deploy slide (the linear mechanism that extends/retracts the intake). */
  public static final class CarriageConfig {
    public static final int CARRIAGE_DEPLOY_MOTOR_ID = 15;

    // ---- Setpoints (ROTOR ROTATIONS of the deploy motor) ----
    // The deploy loop runs on raw rotor rotations (SensorToMechanismRatio = 1), CW positive,
    // travelling 0 -> 15.5. DOWN = deployed, UP = retracted.
    public static final double CARRIAGE_MIN_ROTATIONS  = 0.0;   // retracted hard stop (reverse soft limit)
    public static final double CARRIAGE_MAX_ROTATIONS  = 18.6;  // deployed hard stop (forward soft limit)
    public static final LoggedTunableNumber CARRIAGE_DOWN_POSITION =
        new LoggedTunableNumber("Carriage/DownPosition", 18.00); // extended / deployed
    public static final LoggedTunableNumber CARRIAGE_UP_POSITION =
        new LoggedTunableNumber("Carriage/UpPosition", 0.7); // retracted

    // NOT sim-only: CarriageIOTalonFX uses these to convert rotor rotations <-> meters.
    public static final double CARRIAGE_GEAR_RATIO    = 25.0;
    public static final double CARRIAGE_DRUM_RADIUS_M = 0.020;  // sprocket/pulley radius

    // ---- Deploy MotionMagic loop: MotionMagicVoltage (gains in VOLTS, mechanism in ROTOR ROTATIONS) ----
    public static final LoggedTunableNumber CARRIAGE_KS =
        new LoggedTunableNumber("Carriage/kS", 0.0); // V
    public static final LoggedTunableNumber CARRIAGE_KV =
        new LoggedTunableNumber("Carriage/kV", 0.0); // V / (rot/s)
    public static final LoggedTunableNumber CARRIAGE_KA =
        new LoggedTunableNumber("Carriage/kA", 0.0); // V / (rot/s^2)
    public static final LoggedTunableNumber CARRIAGE_KP =
        new LoggedTunableNumber("Carriage/kP", 5.0); // V / rot
    public static final LoggedTunableNumber CARRIAGE_KI =
        new LoggedTunableNumber("Carriage/kI", 0.0);
    public static final LoggedTunableNumber CARRIAGE_KD =
        new LoggedTunableNumber("Carriage/kD", 0.0); // V / (rot/s)
    public static final LoggedTunableNumber CARRIAGE_MM_CRUISE_MPS =
        new LoggedTunableNumber("Carriage/CruiseVelocity", 50); // rot/s
    public static final LoggedTunableNumber CARRIAGE_MM_ACCEL_MPS2 =
        new LoggedTunableNumber("Carriage/Acceleration", 100); // rot/s^2

    // ---- Pose3d anchor (Mechanism2d / AdvantageScope visualization) ----
    // CARRIAGE_BASE_TRANSLATION = where the slide's zero-position sits in robot frame (m)
    // SLIDE_AXIS                = unit vector along which the carriage extends
    public static final Translation3d CARRIAGE_BASE_TRANSLATION = new Translation3d(0.30, 0.00, 0.10);
    public static final Translation3d SLIDE_AXIS                = new Translation3d(1.0, 0.0, 0.0);
  }

  public static final class RollersConfig {
    public static final int ROLLERS_MOTOR_ID          = 19;
    public static final int ROLLERS_FOLLOWER_MOTOR_ID = 16;  // freed up from the old carriage follower

    public static final LoggedTunableNumber ROLLERS_VELOCITY =
        new LoggedTunableNumber("Rollers/Velocity", 100.0); // rotor rps

    // ---- Velocity loop (VelocityTorqueCurrentFOC, amps/rotor-rps); sim physics in SimConfig ----
    public static final LoggedTunableNumber ROLLERS_KS =
        new LoggedTunableNumber("Rollers/kS", 0.0);
    public static final LoggedTunableNumber ROLLERS_KV =
        new LoggedTunableNumber("Rollers/kV", 0.0);
    public static final LoggedTunableNumber ROLLERS_KP =
        new LoggedTunableNumber("Rollers/kP", 3.0);
    public static final LoggedTunableNumber ROLLERS_KI =
        new LoggedTunableNumber("Rollers/kI", 0.0);
    public static final LoggedTunableNumber ROLLERS_KD =
        new LoggedTunableNumber("Rollers/kD", 0.0);
    public static final LoggedTunableNumber ROLLERS_TORQUE_CURRENT_LIMIT =
        new LoggedTunableNumber("Rollers/TorqueCurrentLimit", 60.0);
  }

  public static final class IndexerConfig {
    public static final int INDEXER_LEADER_MOTOR_ID   = 17;
    public static final int INDEXER_FOLLOWER_MOTOR_ID = 18;

    public static final LoggedTunableNumber INDEXER_VELOCITY =
        new LoggedTunableNumber("Indexer/Velocity", 100.0); // rotor rps

    // ---- Velocity loop (VelocityTorqueCurrentFOC, amps/rotor-rps); sim physics in SimConfig ----
    public static final LoggedTunableNumber INDEXER_KS =
        new LoggedTunableNumber("Indexer/kS", 0.0);
    public static final LoggedTunableNumber INDEXER_KV =
        new LoggedTunableNumber("Indexer/kV", 0.0);
    public static final LoggedTunableNumber INDEXER_KP =
        new LoggedTunableNumber("Indexer/kP", 3.0);
    public static final LoggedTunableNumber INDEXER_KI =
        new LoggedTunableNumber("Indexer/kI", 0.0);
    public static final LoggedTunableNumber INDEXER_KD =
        new LoggedTunableNumber("Indexer/kD", 0.0);
    public static final LoggedTunableNumber INDEXER_TORQUE_CURRENT_LIMIT =
        new LoggedTunableNumber("Indexer/TorqueCurrentLimit", 100.0);
  }

  /**
   * 2026 "Rebuilt" teleop shift schedule: Transition -> Shift1..4 (HUB activity alternates between
   * alliances each shift) -> Endgame (both HUBs active). Durations are official game timing; kept
   * tunable in case the manual numbers get revised. See {@code MatchStateCommand}.
   */
  public static final class MatchStateConfig {
    public static final LoggedTunableNumber TRANSITION_TIME_SEC =
        new LoggedTunableNumber("MatchState/TransitionTimeSec", 10.0);
    public static final LoggedTunableNumber SHIFT_TIME_SEC =
        new LoggedTunableNumber("MatchState/ShiftTimeSec", 25.0); // x4 shifts
    public static final LoggedTunableNumber ENDGAME_TIME_SEC =
        new LoggedTunableNumber("MatchState/EndgameTimeSec", 30.0);
  }

  /**
   * Status LEDs on a CTRE CANdle (see {@code frc.robot.subsystems.leds.Leds}). One solid color per
   * driver action so the driver/coach can read robot state off the strip; picked in priority order
   * (first true wins), highest-priority action first, falling back to the alliance color when idle.
   * CAN ID and every color below are made up -- there's no real hardware/paint-scheme yet, so change
   * the ID to wherever you actually wire the CANdle and the colors to whatever the team likes.
   */
  public static final class LedsConfig {
    public static final int LED_CANDLE_ID = 0;
    public static final int LED_COUNT = 40; // made up -- set to your actual strip length

    // ---- Idle (no driver action held/toggled on) -- alliance color so the strip still means
    // something at rest; falls back to white if disconnected from the FMS/DS (no alliance yet).
    public static final RGBWColor COLOR_IDLE_RED = new RGBWColor(40, 0, 0);
    public static final RGBWColor COLOR_IDLE_BLUE = new RGBWColor(0, 0, 40);
    public static final RGBWColor COLOR_IDLE_NONE = new RGBWColor(40, 40, 40);

    // ---- Per-action colors, in the priority order Leds evaluates them ----
    public static final RGBWColor COLOR_AUTO_SHOOT = new RGBWColor(255, 60, 0); // orange - right trigger
    public static final RGBWColor COLOR_MANUAL_SHOOT = new RGBWColor(0, 80, 255); // blue - left trigger
    public static final RGBWColor COLOR_OUTTAKE = new RGBWColor(255, 0, 0); // red - Y*////
    public static final RGBWColor COLOR_INTAKE_SPIN = new RGBWColor(0, 255, 0); // spring green - B (spin only)
  }

  /**
   * Sim-only physics: the plant models the *IOSim classes feed into WPILib's DCMotorSim /
   * SingleJointedArmSim / ElevatorSim. Nothing here runs on the real robot, so these should never
   * need touching again — kept out of the per-subsystem configs above so those only contain values
   * that matter on hardware. (Ratios the REAL IO uses for feedback/unit conversion — HOOD_GEAR_RATIO,
   * CARRIAGE_GEAR_RATIO, CARRIAGE_DRUM_RADIUS_M — stay in their subsystem configs.)
   */
  public static final class SimConfig {
    // Shooter flywheel (DCMotorSim)
    public static final double SHOOTER_GEAR_RATIO = 1.0;
    public static final double SHOOTER_MOI_KG_M2  = 0.015; // flywheel inertia (estimate)

    // Hood pivot (SingleJointedArmSim)
    public static final double HOOD_LENGTH_M  = 0.25;
    public static final double HOOD_MOI_KG_M2 = 0.05;
    public static final boolean HOOD_SIM_GRAVITY = false; // treat as counter-balanced for a simple loop

    // Intake rollers (DCMotorSim)
    public static final double INTAKE_GEAR_RATIO = 3.0;
    public static final double INTAKE_MOI_KG_M2  = 0.001;

    // Carriage deploy slide (ElevatorSim; gear ratio + drum radius live in CarriageConfig)
    public static final double CARRIAGE_MASS_KG      = 2.0;
    public static final double CARRIAGE_MIN_TRAVEL_M = 0.00;
    public static final double CARRIAGE_MAX_TRAVEL_M = 0.30;
    public static final boolean CARRIAGE_SIM_GRAVITY = false; // horizontal slide

    // Rollers (DCMotorSim)
    public static final double ROLLERS_GEAR_RATIO = 1.0;
    public static final double ROLLERS_MOI_KG_M2  = 0.003;

    // Indexer (DCMotorSim)
    public static final double INDEXER_GEAR_RATIO = 1.0;
    public static final double INDEXER_MOI_KG_M2  = 0.003;
  }
}
