package frc.robot.subsystems.rollers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
  }

  public void setRollersMotorVelocity(double rps) {
    io.setVelocity(rps);
  }
}
