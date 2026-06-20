// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  // AdvantageKit runtime mode. REAL on a roboRIO; SIM in the physics sim; REPLAY when re-running a log.
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode =
      edu.wpi.first.wpilibj.RobotBase.isReal() ? Mode.REAL : simMode;

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

    public static final double SHOOTER_VELOCITY = 100.0;  // rotor rps (note: ~ Kraken FOC free speed, caps ~96 in sim)

    // ---- Sim physics (flywheel = DCMotorSim) + velocity loop (VelocityTorqueCurrentFOC, amps/rotor-rps) ----
    public static final double SHOOTER_GEAR_RATIO = 1.0;
    public static final double SHOOTER_MOI_KG_M2  = 0.015; // flywheel inertia (estimate)
    public static final double SHOOTER_KS = 0.0;   // A  (frictionless sim; tune kS/kV on real hw)
    public static final double SHOOTER_KV = 0.0;   // A / rotor-rps
    public static final double SHOOTER_KP = 8.0;   // A / rotor-rps of error
    public static final double SHOOTER_TORQUE_CURRENT_LIMIT = 80.0; // A
  }

  public static final class IntakeConfig {
    public static final int INTAKE_LEFT_MOTOR_ID    = 13;
    public static final int INTAKE_RIGHT_MOTOR_ID   = 14;
    public static final int INTAKE_DEPLOY_MOTOR_ID  = 15;

    // ---- Setpoints (real-world units at the subsystem API boundary) ----
    public static final double INTAKE_VELOCITY              = 60.0; // roller rps
    public static final double INTAKE_DEPLOY_DOWN_POSITION  = 0.30; // meters extended
    public static final double INTAKE_DEPLOY_UP_POSITION    = 0.00; // meters retracted

    // ---- Sim physics (deploy = linear slide via ElevatorSim) ----
    public static final double DEPLOY_GEAR_RATIO    = 25.0;
    public static final double DEPLOY_DRUM_RADIUS_M = 0.020;  // sprocket/pulley radius
    public static final double DEPLOY_MASS_KG       = 2.0;
    public static final double DEPLOY_MIN_TRAVEL_M  = 0.00;
    public static final double DEPLOY_MAX_TRAVEL_M  = 0.30;
    public static final boolean DEPLOY_SIM_GRAVITY  = false;  // horizontal slide

    // ---- Sim physics (rollers = DCMotorSim) ----
    public static final double ROLLER_GEAR_RATIO = 3.0;
    public static final double ROLLER_MOI_KG_M2  = 0.001;

    // ---- Roller velocity loop: VelocityTorqueCurrentFOC (gains in AMPS per rotor-rps) ----
    // Used by BOTH sim and real — one tuning. The sim plant is frictionless, so kS/kV are 0;
    // the real robot will need kS (and maybe kV) tuned on hardware to hold speed against friction.
    public static final double ROLLER_KS = 0.0;   // A  (static friction)
    public static final double ROLLER_KV = 0.0;   // A / (rotor rps)  (viscous)
    public static final double ROLLER_KP = 0.6;   // A / (rotor rps of error)
    public static final double ROLLER_TORQUE_CURRENT_LIMIT = 60.0; // A, peak |torque current|

    // ---- Deploy MotionMagic loop: MotionMagicVoltage (gains in VOLTS, mechanism in METERS) ----
    public static final double DEPLOY_KS = 0.0;    // V
    public static final double DEPLOY_KV = 24.0;   // V / (m/s)
    public static final double DEPLOY_KA = 0.1;    // V / (m/s^2)
    public static final double DEPLOY_KP = 40.0;   // V / m
    public static final double DEPLOY_KD = 0.0;    // V / (m/s)
    public static final double DEPLOY_MM_CRUISE_MPS = 0.35; // m/s
    public static final double DEPLOY_MM_ACCEL_MPS2 = 1.5;  // m/s^2

    // ---- Pose3d anchor (Mechanism2d / AdvantageScope visualization) ----
    // INTAKE_BASE_TRANSLATION = where the slide's zero-position sits in robot frame (m)
    // SLIDE_AXIS              = unit vector along which the carriage extends
    public static final Translation3d INTAKE_BASE_TRANSLATION = new Translation3d(0.30, 0.00, 0.10);
    public static final Translation3d SLIDE_AXIS              = new Translation3d(1.0, 0.0, 0.0);
    // Roller spin-axis location at deploy=0 (robot frame) for the AdvantageScope roller component;
    // match this to your CAD's model_1 zeroed pivot.
    public static final Translation3d ROLLER_VIS_TRANSLATION  = new Translation3d(0.40, 0.00, 0.08);
  }

  public static final class RollersConfig {
    public static final int ROLLERS_MOTOR_ID = 16;

    public static final double ROLLERS_VELOCITY = 50.0;  // rotor rps

    // ---- Sim physics (DCMotorSim) + velocity loop (VelocityTorqueCurrentFOC, amps/rotor-rps) ----
    public static final double ROLLERS_GEAR_RATIO = 1.0;
    public static final double ROLLERS_MOI_KG_M2  = 0.003;
    public static final double ROLLERS_KS = 0.0;
    public static final double ROLLERS_KV = 0.0;
    public static final double ROLLERS_KP = 0.5;
    public static final double ROLLERS_TORQUE_CURRENT_LIMIT = 60.0;
  }

  public static final class IndexerConfig {
    public static final int INDEXER_LEADER_MOTOR_ID   = 17;
    public static final int INDEXER_FOLLOWER_MOTOR_ID = 18;

    public static final double INDEXER_VELOCITY = 50.0;  // rotor rps

    // ---- Sim physics (DCMotorSim) + velocity loop (VelocityTorqueCurrentFOC, amps/rotor-rps) ----
    public static final double INDEXER_GEAR_RATIO = 1.0;
    public static final double INDEXER_MOI_KG_M2  = 0.003;
    public static final double INDEXER_KS = 0.0;
    public static final double INDEXER_KV = 0.0;
    public static final double INDEXER_KP = 0.5;
    public static final double INDEXER_TORQUE_CURRENT_LIMIT = 60.0;
  }
}
