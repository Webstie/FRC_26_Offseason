package frc.robot.subsystems.intake;

import static frc.robot.Constants.IntakeConfig.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

public class IntakeIOTalonFX implements IntakeIO {

  // protected so IntakeIOSim can drive their Phoenix6 sim states
  protected final TalonFX leftMotor  = new TalonFX(INTAKE_LEFT_MOTOR_ID,  new CANBus("canivore"));
  protected final TalonFX rightMotor = new TalonFX(INTAKE_RIGHT_MOTOR_ID, new CANBus("canivore"));

  private final VelocityTorqueCurrentFOC rollerRequest = new VelocityTorqueCurrentFOC(0).withSlot(0);

  public IntakeIOTalonFX() {
    var rollerCfg = new TalonFXConfiguration();
    // VelocityTorqueCurrentFOC: gains are amps (output) per rotor-rps (error). Shared sim + real.
    rollerCfg.Slot0.kS = INTAKE_KS.get();
    rollerCfg.Slot0.kV = INTAKE_KV.get();
    rollerCfg.Slot0.kP = INTAKE_KP.get();
    rollerCfg.TorqueCurrent.PeakForwardTorqueCurrent =  INTAKE_TORQUE_CURRENT_LIMIT.get();
    rollerCfg.TorqueCurrent.PeakReverseTorqueCurrent = -INTAKE_TORQUE_CURRENT_LIMIT.get();
    rollerCfg.CurrentLimits.SupplyCurrentLimit = INTAKE_SUPPLY_CURRENT_LIMIT.get();
    rollerCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    leftMotor.getConfigurator().apply(rollerCfg);
    rightMotor.getConfigurator().apply(rollerCfg);

    rightMotor.setControl(new Follower(leftMotor.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.rollerVelocityRPS  = leftMotor.getVelocity().getValueAsDouble();
    inputs.rollerPositionRot  = leftMotor.getPosition().getValueAsDouble();
    inputs.rollerAppliedVolts = leftMotor.getMotorVoltage().getValueAsDouble();
    inputs.rollerCurrentAmps  = leftMotor.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public void setRollerVelocity(double rps) {
    leftMotor.setControl(rollerRequest.withVelocity(rps));
  }

  @Override
  public void setGains(double kP, double kI, double kD, double kS, double kV) {
    var slot = new SlotConfigs();
    slot.kP = kP;
    slot.kI = kI;
    slot.kD = kD;
    slot.kS = kS;
    slot.kV = kV;
    leftMotor.getConfigurator().apply(slot);
    rightMotor.getConfigurator().apply(slot);
  }

  @Override
  public void setCurrentLimit(double amps) {
    var limit = new TorqueCurrentConfigs();
    limit.PeakForwardTorqueCurrent = amps;
    limit.PeakReverseTorqueCurrent = -amps;
    leftMotor.getConfigurator().apply(limit);
    rightMotor.getConfigurator().apply(limit);
  }

  @Override
  public void setSupplyCurrentLimit(double amps) {
    var limit = new CurrentLimitsConfigs();
    limit.SupplyCurrentLimit = amps;
    limit.SupplyCurrentLimitEnable = true;
    leftMotor.getConfigurator().apply(limit);
    rightMotor.getConfigurator().apply(limit);
  }
}
