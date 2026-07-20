package frc.robot.subsystems.intake;

import static frc.robot.Constants.IntakeConfig.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/** Intake rollers. The deploy slide is the separate {@code Carriage} subsystem. */
public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private boolean running = false;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    // Re-apply gains to the roller controllers whenever any tunable changes.
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> io.setGains(INTAKE_KP.get(), INTAKE_KI.get(), INTAKE_KD.get(), INTAKE_KS.get(), INTAKE_KV.get()),
        INTAKE_KP, INTAKE_KI, INTAKE_KD, INTAKE_KS, INTAKE_KV);

    LoggedTunableNumber.ifChanged(
        hashCode() + 1,
        () -> io.setCurrentLimit(INTAKE_TORQUE_CURRENT_LIMIT.get()),
        INTAKE_TORQUE_CURRENT_LIMIT);
  }

  /** Spin the rollers forward (intake in) at the tunable setpoint. */
  public void spin() {
    setSpinning(true, INTAKE_VELOCITY.get());
  }

  /** Spin the rollers backward (spit out) at the tunable setpoint. */
  public void spinReverse() {
    setSpinning(true, -INTAKE_VELOCITY.get());
  }

  public void stop() {
    setSpinning(false, 0);
  }

  private void setSpinning(boolean spinning, double velocityRps) {
    running = spinning;
    setIntakeMotorVelocity(velocityRps);
  }

  /** True if the rollers were last commanded to spin (either direction). */
  public boolean isRunning() {
    return running;
  }

  public Command runCommand() {
    return runOnce(this::spin);
  }

  public Command reverseCommand() {
    return runOnce(this::spinReverse);
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }

  public void setIntakeMotorVelocity(double rps) {
    io.setRollerVelocity(rps);
  }
}
