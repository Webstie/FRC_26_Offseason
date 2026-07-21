package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {

  @AutoLog
  class IndexerIOInputs {
    public double velocityRPS = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double motor2VelocityRPS = 0.0;
    public double motor2AppliedVolts = 0.0;
    public double motor2CurrentAmps = 0.0;
  }

  default void updateInputs(IndexerIOInputs inputs) {}

  /** Run the indexer at the given rotor velocity (rps). */
  default void setVelocity(double rps) {}

  /** Live-update the indexer velocity-loop gains (Slot0) for tuning. */
  default void setGains(double kP, double kI, double kD, double kS, double kV) {}

  /** Live-update the peak torque-current limit (amps). */
  default void setCurrentLimit(double amps) {}

  /** Live-update the supply-side (battery-draw) current limit (amps). */
  default void setSupplyCurrentLimit(double amps) {}
}
