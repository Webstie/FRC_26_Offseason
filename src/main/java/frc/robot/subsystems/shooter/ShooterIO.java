package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  class ShooterIOInputs {
    public double velocityRPS = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;

    // Hood pivot (position-controlled, fused CANcoder; mechanism rotations).
    public double hoodPositionRotations = 0.0;
    public double hoodVelocityRotPerSec = 0.0;
    public double hoodAppliedVolts = 0.0;
    public double hoodCurrentAmps = 0.0;
  }

  default void updateInputs(ShooterIOInputs inputs) {}

  /** Run the shooter flywheel at the given rotor velocity (rps) with the closed velocity loop. */
  default void setVelocity(double rps) {}

  /** Drive the hood pivot to the given position (mechanism rotations), MotionMagic position control. */
  default void setHoodPosition(double rotations) {}

  /** Live-update the flywheel velocity-loop gains (Slot0) for tuning. */
  default void setGains(double kP, double kI, double kD, double kS, double kV) {}

  /** Live-update the flywheel peak torque-current limit (amps). */
  default void setCurrentLimit(double amps) {}

  /** Live-update the flywheel supply-side (battery-draw) current limit (amps). */
  default void setSupplyCurrentLimit(double amps) {}

  /** Live-update the hood MotionMagicVoltage gains (volts) for tuning. */
  default void setHoodGains(double kP, double kI, double kD, double kS, double kV, double kA) {}

  /** Live-update the hood MotionMagic cruise velocity (rot/s) and acceleration (rot/s^2). */
  default void setHoodMotionMagicConstraints(double cruiseVelocity, double acceleration) {}

  /** Coast the flywheel: put it in neutral so it spins down on its own inertia (NeutralMode=Coast). */
  default void stop() {}
}
