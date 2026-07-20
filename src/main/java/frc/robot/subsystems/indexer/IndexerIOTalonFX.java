package frc.robot.subsystems.indexer;

import static frc.robot.Constants.IndexerConfig.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

public class IndexerIOTalonFX implements IndexerIO {

  // protected so IndexerIOSim can drive their Phoenix6 sim states
  protected final TalonFX leaderMotor   = new TalonFX(INDEXER_LEADER_MOTOR_ID,   new CANBus("canivore"));
  protected final TalonFX followerMotor = new TalonFX(INDEXER_FOLLOWER_MOTOR_ID, new CANBus("canivore"));
  private final VelocityTorqueCurrentFOC request = new VelocityTorqueCurrentFOC(0).withSlot(0);

  public IndexerIOTalonFX() {
    var cfg = new TalonFXConfiguration();
    cfg.Slot0.kS = INDEXER_KS.get();
    cfg.Slot0.kV = INDEXER_KV.get();
    cfg.Slot0.kP = INDEXER_KP.get();
    cfg.TorqueCurrent.PeakForwardTorqueCurrent =  INDEXER_TORQUE_CURRENT_LIMIT.get();
    cfg.TorqueCurrent.PeakReverseTorqueCurrent = -INDEXER_TORQUE_CURRENT_LIMIT.get();
    leaderMotor.getConfigurator().apply(cfg);
    followerMotor.getConfigurator().apply(cfg);

    followerMotor.setControl(new Follower(leaderMotor.getDeviceID(), MotorAlignmentValue.Aligned));
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.velocityRPS  = leaderMotor.getVelocity().getValueAsDouble();
    inputs.appliedVolts = leaderMotor.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps  = leaderMotor.getStatorCurrent().getValueAsDouble();
    // Follower telemetry (motor2* names kept stable across the control-mode experiments): current
    // opposite in sign to the leader (or stuck at zero) = alignment/CAN problem.
    inputs.motor2VelocityRPS  = followerMotor.getVelocity().getValueAsDouble();
    inputs.motor2AppliedVolts = followerMotor.getMotorVoltage().getValueAsDouble();
    inputs.motor2CurrentAmps  = followerMotor.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public void setVelocity(double rps) {
    // Follower mirrors the leader's output automatically.
    leaderMotor.setControl(request.withVelocity(rps));
  }

  @Override
  public void setGains(double kP, double kI, double kD, double kS, double kV) {
    var slot0 = new Slot0Configs();
    slot0.kP = kP;
    slot0.kI = kI;
    slot0.kD = kD;
    slot0.kS = kS;
    slot0.kV = kV;
    // Only the leader runs the velocity loop, but keep both configs identical.
    leaderMotor.getConfigurator().apply(slot0);
    followerMotor.getConfigurator().apply(slot0);
  }

  @Override
  public void setCurrentLimit(double amps) {
    var limit = new TorqueCurrentConfigs();
    limit.PeakForwardTorqueCurrent = amps;
    limit.PeakReverseTorqueCurrent = -amps;
    leaderMotor.getConfigurator().apply(limit);
    followerMotor.getConfigurator().apply(limit);
  }
}
