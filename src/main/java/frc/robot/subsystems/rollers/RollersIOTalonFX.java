package frc.robot.subsystems.rollers;

import static frc.robot.Constants.RollersConfig.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

public class RollersIOTalonFX implements RollersIO {

  protected final TalonFX motor = new TalonFX(ROLLERS_MOTOR_ID, new CANBus("canivore"));
  private final VelocityTorqueCurrentFOC request = new VelocityTorqueCurrentFOC(0).withSlot(0);

  public RollersIOTalonFX() {
    var cfg = new TalonFXConfiguration();
    cfg.Slot0.kS = ROLLERS_KS;
    cfg.Slot0.kV = ROLLERS_KV;
    cfg.Slot0.kP = ROLLERS_KP;
    cfg.TorqueCurrent.PeakForwardTorqueCurrent =  ROLLERS_TORQUE_CURRENT_LIMIT;
    cfg.TorqueCurrent.PeakReverseTorqueCurrent = -ROLLERS_TORQUE_CURRENT_LIMIT;
    motor.getConfigurator().apply(cfg);
  }

  @Override
  public void updateInputs(RollersIOInputs inputs) {
    inputs.velocityRPS  = motor.getVelocity().getValueAsDouble();
    inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps  = motor.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public void setVelocity(double rps) {
    motor.setControl(request.withVelocity(rps));
  }

  @Override
  public void stop() {
    motor.setControl(request.withVelocity(0));
  }
}
