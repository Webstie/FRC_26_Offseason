package frc.robot.subsystems.carriage;

import org.littletonrobotics.junction.AutoLog;

public interface CarriageIO {

  @AutoLog
  class CarriageIOInputs {
    public double deployPositionRotations = 0.0;
    public double deployVelocityRotPerSec = 0.0;
    public double deployAppliedVolts = 0.0;
    public double deployCurrentAmps = 0.0;
  }

  default void updateInputs(CarriageIOInputs inputs) {}

  /** Drive the carriage to the given position (rotor rotations). */
  default void setDeployPosition(double rotations) {}

  /** Live-update the MotionMagicVoltage gains (volts). */
  default void setGains(double kP, double kI, double kD, double kS, double kV, double kA) {}

  /** Live-update the MotionMagic cruise velocity (rot/s) and acceleration (rot/s^2). */
  default void setMotionMagicConstraints(double cruiseVelocity, double acceleration) {}
}
