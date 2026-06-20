package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollersIO {

  @AutoLog
  class RollersIOInputs {
    public double velocityRPS = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  default void updateInputs(RollersIOInputs inputs) {}

  /** Run the rollers at the given rotor velocity (rps). */
  default void setVelocity(double rps) {}

  default void stop() {}
}
