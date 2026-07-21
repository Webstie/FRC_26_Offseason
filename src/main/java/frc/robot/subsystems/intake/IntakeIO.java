package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  class IntakeIOInputs {
    public double rollerVelocityRPS = 0.0;
    public double rollerPositionRot = 0.0;
    public double rollerAppliedVolts = 0.0;
    public double rollerCurrentAmps = 0.0;
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  /** Run the intake rollers at the given velocity (rps). */
  default void setRollerVelocity(double rps) {}

  /** Live-update the velocity-loop gains (amps per rotor-rps). */
  default void setGains(double kP, double kI, double kD, double kS, double kV) {}

  /** Live-update the peak torque-current limit (amps). */
  default void setCurrentLimit(double amps) {}

  /** Live-update the supply-side (battery-draw) current limit (amps). */
  default void setSupplyCurrentLimit(double amps) {}
}
