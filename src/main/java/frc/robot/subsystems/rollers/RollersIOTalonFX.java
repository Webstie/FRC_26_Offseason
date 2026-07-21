package frc.robot.subsystems.rollers;

import static frc.robot.Constants.RollersConfig.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

public class RollersIOTalonFX implements RollersIO {

  protected final TalonFX motor = new TalonFX(ROLLERS_MOTOR_ID, new CANBus("rio"));
  protected final TalonFX followerMotor =
      new TalonFX(ROLLERS_FOLLOWER_MOTOR_ID, new CANBus("rio"));
  private final VelocityTorqueCurrentFOC request = new VelocityTorqueCurrentFOC(0).withSlot(0);

  public RollersIOTalonFX() {
    var cfg = new TalonFXConfiguration();
    cfg.Slot0.kS = ROLLERS_KS.get();
    cfg.Slot0.kV = ROLLERS_KV.get();
    cfg.Slot0.kP = ROLLERS_KP.get();
    cfg.TorqueCurrent.PeakForwardTorqueCurrent =  ROLLERS_TORQUE_CURRENT_LIMIT.get();
    cfg.TorqueCurrent.PeakReverseTorqueCurrent = -ROLLERS_TORQUE_CURRENT_LIMIT.get();
    cfg.CurrentLimits.SupplyCurrentLimit = ROLLERS_SUPPLY_CURRENT_LIMIT.get();
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motor.getConfigurator().apply(cfg);
    followerMotor.getConfigurator().apply(cfg);

    // Second roller follows the leader (mirror its output). Correct once the two are mechanically
    // coupled; until then they'll appear to run at different speeds. Opposed = counter-rotate to grip.
    followerMotor.setControl(new Follower(motor.getDeviceID(), MotorAlignmentValue.Opposed));
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
  public void setGains(double kP, double kI, double kD, double kS, double kV) {
    var slot0 = new Slot0Configs();
    slot0.kP = kP;
    slot0.kI = kI;
    slot0.kD = kD;
    slot0.kS = kS;
    slot0.kV = kV;
    motor.getConfigurator().apply(slot0);
  }

  @Override
  public void setCurrentLimit(double amps) {
    var limit = new TorqueCurrentConfigs();
    limit.PeakForwardTorqueCurrent = amps;
    limit.PeakReverseTorqueCurrent = -amps;
    motor.getConfigurator().apply(limit);
  }

  @Override
  public void setSupplyCurrentLimit(double amps) {
    var limit = new CurrentLimitsConfigs();
    limit.SupplyCurrentLimit = amps;
    limit.SupplyCurrentLimitEnable = true;
    motor.getConfigurator().apply(limit);
  }
}
