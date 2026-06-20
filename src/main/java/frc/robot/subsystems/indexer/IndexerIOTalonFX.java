package frc.robot.subsystems.indexer;

import static frc.robot.Constants.IndexerConfig.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

public class IndexerIOTalonFX implements IndexerIO {

  protected final TalonFX leaderMotor   = new TalonFX(INDEXER_LEADER_MOTOR_ID,   new CANBus("canivore"));
  protected final TalonFX followerMotor = new TalonFX(INDEXER_FOLLOWER_MOTOR_ID, new CANBus("canivore"));
  private final VelocityTorqueCurrentFOC request = new VelocityTorqueCurrentFOC(0).withSlot(0);

  public IndexerIOTalonFX() {
    var cfg = new TalonFXConfiguration();
    cfg.Slot0.kS = INDEXER_KS;
    cfg.Slot0.kV = INDEXER_KV;
    cfg.Slot0.kP = INDEXER_KP;
    cfg.TorqueCurrent.PeakForwardTorqueCurrent =  INDEXER_TORQUE_CURRENT_LIMIT;
    cfg.TorqueCurrent.PeakReverseTorqueCurrent = -INDEXER_TORQUE_CURRENT_LIMIT;
    leaderMotor.getConfigurator().apply(cfg);
    followerMotor.getConfigurator().apply(cfg);

    followerMotor.setControl(new Follower(leaderMotor.getDeviceID(), MotorAlignmentValue.Aligned));
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.velocityRPS  = leaderMotor.getVelocity().getValueAsDouble();
    inputs.appliedVolts = leaderMotor.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps  = leaderMotor.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public void setVelocity(double rps) {
    leaderMotor.setControl(request.withVelocity(rps));
  }

  @Override
  public void stop() {
    leaderMotor.setControl(request.withVelocity(0));
  }
}
