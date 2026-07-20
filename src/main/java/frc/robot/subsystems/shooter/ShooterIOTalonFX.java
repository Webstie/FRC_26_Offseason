package frc.robot.subsystems.shooter;

import static frc.robot.Constants.ShooterConfig.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;

public class ShooterIOTalonFX implements ShooterIO {

  protected final TalonFX leftUpMotor    = new TalonFX(SHOOTER_LEFT_UP_MOTOR_ID,    new CANBus("canivore"));
  protected final TalonFX leftDownMotor  = new TalonFX(SHOOTER_LEFT_DOWN_MOTOR_ID,  new CANBus("canivore"));
  protected final TalonFX rightUpMotor   = new TalonFX(SHOOTER_RIGHT_UP_MOTOR_ID,   new CANBus("canivore"));
  protected final TalonFX rightDownMotor = new TalonFX(SHOOTER_RIGHT_DOWN_MOTOR_ID, new CANBus("canivore"));

  // Hood pivot: one TalonFX fused with one CANcoder mounted on the hood axis.
  // FusedCANcoder REQUIRES the CANcoder and the Talon FX to be on the SAME CAN bus, so both are on
  // "canivore". (The CANcoder must also be physically wired into the CANivore CAN chain.)
  protected final TalonFX hoodMotor     = new TalonFX(SHOOTER_HOOD_MOTOR_ID,    new CANBus("canivore"));
  protected final CANcoder hoodCancoder = new CANcoder(SHOOTER_HOOD_CANCODER_ID, new CANBus("canivore"));

  private final VelocityTorqueCurrentFOC request = new VelocityTorqueCurrentFOC(0).withSlot(0);
  private final MotionMagicVoltage hoodRequest = new MotionMagicVoltage(0).withSlot(0);
  private final NeutralOut neutralRequest = new NeutralOut();

  public ShooterIOTalonFX() {
    var cfg = new TalonFXConfiguration();
    cfg.Slot0.kS = SHOOTER_KS.get();
    cfg.Slot0.kV = SHOOTER_KV.get();
    cfg.Slot0.kP = SHOOTER_KP.get();
    cfg.TorqueCurrent.PeakForwardTorqueCurrent =  SHOOTER_TORQUE_CURRENT_LIMIT.get();
    cfg.TorqueCurrent.PeakReverseTorqueCurrent = -SHOOTER_TORQUE_CURRENT_LIMIT.get();
    // Flywheel coasts in neutral (on stop() and when disabled) so it spins down on its own MOI
    // instead of the motors braking it.
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Reverse the whole flywheel by inverting the LEADER. Inverting flips BOTH the output and the
    // velocity sensor, so commanding +rps spins the desired direction AND getVelocity() stays
    // positive -> atVelocity() works. (Negating the setpoint instead made the sensor read -rps while
    // the check compared against +rps, so "at speed" never went true.) The followers track the
    // leader via their Follower alignment below, so inverting the leader reverses all four.
    cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    leftUpMotor.getConfigurator().apply(cfg);

    cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // followers: default orientation
    leftDownMotor.getConfigurator().apply(cfg);
    rightUpMotor.getConfigurator().apply(cfg);
    rightDownMotor.getConfigurator().apply(cfg);

    int leaderID = leftUpMotor.getDeviceID();
    leftDownMotor.setControl(new Follower(leaderID, MotorAlignmentValue.Aligned));
    rightUpMotor.setControl(new Follower(leaderID, MotorAlignmentValue.Opposed));
    rightDownMotor.setControl(new Follower(leaderID, MotorAlignmentValue.Opposed));

    // ---- Hood CANcoder: reports the hood angle directly (1:1 with the mechanism) ----
    var ccCfg = new CANcoderConfiguration();
    ccCfg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    ccCfg.MagnetSensor.MagnetOffset = HOOD_CANCODER_OFFSET_ROT;
    hoodCancoder.getConfigurator().apply(ccCfg);

    // ---- Hood motor: MotionMagicVoltage position loop, FusedCANcoder feedback ----
    // With FusedCANcoder + SensorToMechanismRatio = 1, getPosition() reads HOOD rotations directly,
    // so setHoodPosition() commands mechanism rotations with no unit conversion.
    var hoodCfg = new TalonFXConfiguration();
    hoodCfg.Slot0.kS = HOOD_KS.get();
    hoodCfg.Slot0.kV = HOOD_KV.get();
    hoodCfg.Slot0.kA = HOOD_KA.get();
    hoodCfg.Slot0.kP = HOOD_KP.get();
    hoodCfg.Slot0.kI = HOOD_KI.get();
    hoodCfg.Slot0.kD = HOOD_KD.get();
    hoodCfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    hoodCfg.MotionMagic.MotionMagicCruiseVelocity = HOOD_MM_CRUISE_RPS.get();
    hoodCfg.MotionMagic.MotionMagicAcceleration   = HOOD_MM_ACCEL_RPS2.get();
    hoodCfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    hoodCfg.Feedback.FeedbackRemoteSensorID = hoodCancoder.getDeviceID();
    hoodCfg.Feedback.RotorToSensorRatio = HOOD_GEAR_RATIO; // rotor rotations per hood (CANcoder) rotation
    hoodCfg.Feedback.SensorToMechanismRatio = 1.0;         // CANcoder is on the hood axis
    hoodCfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    hoodCfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    hoodCfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = HOOD_MAX_ROTATIONS;
    hoodCfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = HOOD_MIN_ROTATIONS;
    hoodMotor.getConfigurator().apply(hoodCfg);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.velocityRPS  = leftUpMotor.getVelocity().getValueAsDouble();
    inputs.appliedVolts = leftUpMotor.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps  = leftUpMotor.getStatorCurrent().getValueAsDouble();

    inputs.hoodPositionRotations = hoodMotor.getPosition().getValueAsDouble();
    inputs.hoodVelocityRotPerSec = hoodMotor.getVelocity().getValueAsDouble();
    inputs.hoodAppliedVolts      = hoodMotor.getMotorVoltage().getValueAsDouble();
    inputs.hoodCurrentAmps       = hoodMotor.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public void setVelocity(double rps) {
    leftUpMotor.setControl(request.withVelocity(rps));
  }

  @Override
  public void stop() {
    // Neutral the leader; with NeutralMode=Coast it (and the followers tracking it) free-spin down.
    leftUpMotor.setControl(neutralRequest);
  }

  @Override
  public void setHoodPosition(double rotations) {
    // Direction-independent travel limit: clamp the commanded position to the hood's range in
    // POSITION space. Unlike the TalonFX hardware soft limits (which hard-assume positive output
    // increases sensor position), this works for ANY motor-inversion / CANcoder-direction combo,
    // so the limit holds without touching the configured directions.
    double clamped = MathUtil.clamp(rotations, HOOD_MIN_ROTATIONS, HOOD_MAX_ROTATIONS);
    hoodMotor.setControl(hoodRequest.withPosition(clamped));
  }

  @Override
  public void setGains(double kP, double kI, double kD, double kS, double kV) {
    // Only the leader runs the velocity loop; followers just mirror its output.
    var slot0 = new Slot0Configs();
    slot0.kP = kP;
    slot0.kI = kI;
    slot0.kD = kD;
    slot0.kS = kS;
    slot0.kV = kV;
    leftUpMotor.getConfigurator().apply(slot0);
  }

  @Override
  public void setCurrentLimit(double amps) {
    var limit = new TorqueCurrentConfigs();
    limit.PeakForwardTorqueCurrent = amps;
    limit.PeakReverseTorqueCurrent = -amps;
    leftUpMotor.getConfigurator().apply(limit);
  }

  @Override
  public void setHoodGains(double kP, double kI, double kD, double kS, double kV, double kA) {
    var slot0 = new Slot0Configs();
    slot0.kP = kP;
    slot0.kI = kI;
    slot0.kD = kD;
    slot0.kS = kS;
    slot0.kV = kV;
    slot0.kA = kA;
    hoodMotor.getConfigurator().apply(slot0);
  }

  @Override
  public void setHoodMotionMagicConstraints(double cruiseVelocity, double acceleration) {
    var mm = new MotionMagicConfigs();
    mm.MotionMagicCruiseVelocity = cruiseVelocity;
    mm.MotionMagicAcceleration = acceleration;
    hoodMotor.getConfigurator().apply(mm);
  }
}
