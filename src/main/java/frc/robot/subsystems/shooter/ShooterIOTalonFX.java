package frc.robot.subsystems.shooter;

import static frc.robot.Constants.ShooterConfig.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

public class ShooterIOTalonFX implements ShooterIO {

  protected final TalonFX leftUpMotor    = new TalonFX(SHOOTER_LEFT_UP_MOTOR_ID,    new CANBus("canivore"));
  protected final TalonFX leftDownMotor  = new TalonFX(SHOOTER_LEFT_DOWN_MOTOR_ID,  new CANBus("canivore"));
  protected final TalonFX rightUpMotor   = new TalonFX(SHOOTER_RIGHT_UP_MOTOR_ID,   new CANBus("canivore"));
  protected final TalonFX rightDownMotor = new TalonFX(SHOOTER_RIGHT_DOWN_MOTOR_ID, new CANBus("canivore"));

  private final VelocityTorqueCurrentFOC request = new VelocityTorqueCurrentFOC(0).withSlot(0);

  public ShooterIOTalonFX() {
    var cfg = new TalonFXConfiguration();
    cfg.Slot0.kS = SHOOTER_KS;
    cfg.Slot0.kV = SHOOTER_KV;
    cfg.Slot0.kP = SHOOTER_KP;
    cfg.TorqueCurrent.PeakForwardTorqueCurrent =  SHOOTER_TORQUE_CURRENT_LIMIT;
    cfg.TorqueCurrent.PeakReverseTorqueCurrent = -SHOOTER_TORQUE_CURRENT_LIMIT;
    leftUpMotor.getConfigurator().apply(cfg);
    leftDownMotor.getConfigurator().apply(cfg);
    rightUpMotor.getConfigurator().apply(cfg);
    rightDownMotor.getConfigurator().apply(cfg);

    int leaderID = leftUpMotor.getDeviceID();
    leftDownMotor.setControl(new Follower(leaderID, MotorAlignmentValue.Aligned));
    rightUpMotor.setControl(new Follower(leaderID, MotorAlignmentValue.Opposed));
    rightDownMotor.setControl(new Follower(leaderID, MotorAlignmentValue.Opposed));
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.velocityRPS  = leftUpMotor.getVelocity().getValueAsDouble();
    inputs.appliedVolts = leftUpMotor.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps  = leftUpMotor.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public void setVelocity(double rps) {
    leftUpMotor.setControl(request.withVelocity(rps));
  }

  @Override
  public void stop() {
    leftUpMotor.setControl(request.withVelocity(0));
  }
}
