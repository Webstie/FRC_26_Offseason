package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  class IntakeIOInputs {
    public double rollerVelocityRPS = 0.0;
    public double rollerPositionRot = 0.0;
    public double rollerAppliedVolts = 0.0;
    public double rollerCurrentAmps = 0.0;

    public double deployPositionMeters = 0.0;
    public double deployVelocityMPS = 0.0;
    public double deployAppliedVolts = 0.0;
    public double deployCurrentAmps = 0.0;
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  /** Run the intake rollers at the given velocity (rps). */
  default void setRollerVelocity(double rps) {}

  /** Drive the deploy carriage to the given linear position (meters). */
  default void setDeployPosition(double meters) {}

  default void stopRoller() {}
}
