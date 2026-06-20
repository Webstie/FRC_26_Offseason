package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  class ShooterIOInputs {
    public double velocityRPS = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  default void updateInputs(ShooterIOInputs inputs) {}

  /** Run the shooter flywheel at the given rotor velocity (rps). */
  default void setVelocity(double rps) {}

  default void stop() {}
}
