package frc.robot.subsystems.rollers;

import static frc.robot.Constants.RollersConfig.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Rollers extends SubsystemBase {

  private final RollersIO io;
  private final RollersIOInputsAutoLogged inputs = new RollersIOInputsAutoLogged();

  public Rollers(RollersIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Rollers", inputs);

    // Re-apply gains to the motor controller whenever any tunable changes.
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> io.setGains(ROLLERS_KP.get(), ROLLERS_KI.get(), ROLLERS_KD.get(), ROLLERS_KS.get(), ROLLERS_KV.get()),
        ROLLERS_KP, ROLLERS_KI, ROLLERS_KD, ROLLERS_KS, ROLLERS_KV);

    LoggedTunableNumber.ifChanged(
        hashCode() + 1,
        () -> io.setCurrentLimit(ROLLERS_TORQUE_CURRENT_LIMIT.get()),
        ROLLERS_TORQUE_CURRENT_LIMIT);
  }

  public void setRollersMotorVelocity(double rps) {
    io.setVelocity(rps);
  }
}
