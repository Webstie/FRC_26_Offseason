package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {

  @AutoLog
  class IndexerIOInputs {
    public double velocityRPS = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  default void updateInputs(IndexerIOInputs inputs) {}

  /** Run the indexer at the given rotor velocity (rps). */
  default void setVelocity(double rps) {}

  default void stop() {}
}
