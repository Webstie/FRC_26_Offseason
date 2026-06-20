package frc.robot.subsystems.intake;

import static frc.robot.Constants.IntakeConfig.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

public class IntakeIOTalonFX implements IntakeIO {

  // rotor rotations per meter of linear travel (protected: the sim subclass converts with it)
  protected static final double ROTOR_PER_METER =
      DEPLOY_GEAR_RATIO / (2.0 * Math.PI * DEPLOY_DRUM_RADIUS_M);

  // protected so IntakeIOSim can drive their Phoenix6 sim states
  protected final TalonFX leftMotor   = new TalonFX(INTAKE_LEFT_MOTOR_ID,   new CANBus("canivore"));
  protected final TalonFX rightMotor  = new TalonFX(INTAKE_RIGHT_MOTOR_ID,  new CANBus("canivore"));
  protected final TalonFX deployMotor = new TalonFX(INTAKE_DEPLOY_MOTOR_ID, new CANBus("canivore"));

  private final VelocityTorqueCurrentFOC rollerRequest = new VelocityTorqueCurrentFOC(0).withSlot(0);
  private final MotionMagicVoltage deployRequest = new MotionMagicVoltage(0).withSlot(0);

  public IntakeIOTalonFX() {
    var rollerCfg = new TalonFXConfiguration();
    // VelocityTorqueCurrentFOC: gains are amps (output) per rotor-rps (error). Shared sim + real.
    rollerCfg.Slot0.kS = ROLLER_KS;
    rollerCfg.Slot0.kV = ROLLER_KV;
    rollerCfg.Slot0.kP = ROLLER_KP;
    rollerCfg.TorqueCurrent.PeakForwardTorqueCurrent =  ROLLER_TORQUE_CURRENT_LIMIT;
    rollerCfg.TorqueCurrent.PeakReverseTorqueCurrent = -ROLLER_TORQUE_CURRENT_LIMIT;
    leftMotor.getConfigurator().apply(rollerCfg);
    rightMotor.getConfigurator().apply(rollerCfg);

    var deployCfg = new TalonFXConfiguration();
    // MotionMagicVoltage: gains are volts; mechanism reads/commands in METERS (SensorToMechanismRatio).
    deployCfg.Slot0.kS = DEPLOY_KS;
    deployCfg.Slot0.kV = DEPLOY_KV;
    deployCfg.Slot0.kA = DEPLOY_KA;
    deployCfg.Slot0.kP = DEPLOY_KP;
    deployCfg.Slot0.kD = DEPLOY_KD;
    deployCfg.MotionMagic.MotionMagicCruiseVelocity = DEPLOY_MM_CRUISE_MPS;
    deployCfg.MotionMagic.MotionMagicAcceleration   = DEPLOY_MM_ACCEL_MPS2;
    // Make getPosition()/setpoints read out in METERS directly
    deployCfg.Feedback.SensorToMechanismRatio = ROTOR_PER_METER;
    deployMotor.getConfigurator().apply(deployCfg);

    rightMotor.setControl(new Follower(leftMotor.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.rollerVelocityRPS  = leftMotor.getVelocity().getValueAsDouble();
    inputs.rollerPositionRot  = leftMotor.getPosition().getValueAsDouble();
    inputs.rollerAppliedVolts = leftMotor.getMotorVoltage().getValueAsDouble();
    inputs.rollerCurrentAmps  = leftMotor.getStatorCurrent().getValueAsDouble();

    inputs.deployPositionMeters = deployMotor.getPosition().getValueAsDouble();
    inputs.deployVelocityMPS    = deployMotor.getVelocity().getValueAsDouble();
    inputs.deployAppliedVolts   = deployMotor.getMotorVoltage().getValueAsDouble();
    inputs.deployCurrentAmps    = deployMotor.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public void setRollerVelocity(double rps) {
    leftMotor.setControl(rollerRequest.withVelocity(rps));
  }

  @Override
  public void setDeployPosition(double meters) {
    deployMotor.setControl(deployRequest.withPosition(meters));
  }

  @Override
  public void stopRoller() {
    leftMotor.setControl(rollerRequest.withVelocity(0));
  }
}
