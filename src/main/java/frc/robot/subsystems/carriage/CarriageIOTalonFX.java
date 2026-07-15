package frc.robot.subsystems.carriage;

import static frc.robot.Constants.CarriageConfig.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class CarriageIOTalonFX implements CarriageIO {

  // rotor rotations per meter of linear travel (protected: the sim subclass converts with it)
  protected static final double ROTOR_PER_METER =
      CARRIAGE_GEAR_RATIO / (2.0 * Math.PI * CARRIAGE_DRUM_RADIUS_M);

  // protected so CarriageIOSim can drive its Phoenix6 sim state
  protected final TalonFX deployMotor =
      new TalonFX(CARRIAGE_DEPLOY_MOTOR_ID, new CANBus("rio"));

  private final MotionMagicVoltage deployRequest = new MotionMagicVoltage(0).withSlot(0);

  public CarriageIOTalonFX() {
    var deployCfg = new TalonFXConfiguration();
    // MotionMagicVoltage: gains are volts; mechanism reads/commands in raw ROTOR ROTATIONS.
    deployCfg.Slot0.kS = CARRIAGE_KS.get();
    deployCfg.Slot0.kV = CARRIAGE_KV.get();
    deployCfg.Slot0.kA = CARRIAGE_KA.get();
    deployCfg.Slot0.kP = CARRIAGE_KP.get();
    deployCfg.Slot0.kD = CARRIAGE_KD.get();
    deployCfg.MotionMagic.MotionMagicCruiseVelocity = CARRIAGE_MM_CRUISE_MPS.get();
    deployCfg.MotionMagic.MotionMagicAcceleration   = CARRIAGE_MM_ACCEL_MPS2.get();
    // Deploy motor spins CW+ for extend; brake so it holds position when disabled/at target.
    deployCfg.MotorOutput.Inverted   = InvertedValue.Clockwise_Positive;
    deployCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // Raw rotor rotations (no unit conversion) so setpoints/soft-limits are the 0..15.5 rotor scale.
    deployCfg.Feedback.SensorToMechanismRatio = 1.0;
    // Soft limits keep the slide inside its travel.
    deployCfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    deployCfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = CARRIAGE_MAX_ROTATIONS;
    deployCfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    deployCfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = CARRIAGE_MIN_ROTATIONS;
    deployMotor.getConfigurator().apply(deployCfg);
  }

  @Override
  public void setGains(double kP, double kI, double kD, double kS, double kV, double kA) {
    var slot = new SlotConfigs();
    slot.kP = kP;
    slot.kI = kI;
    slot.kD = kD;
    slot.kS = kS;
    slot.kV = kV;
    slot.kA = kA;
    deployMotor.getConfigurator().apply(slot);
  }

  @Override
  public void setMotionMagicConstraints(double cruiseVelocity, double acceleration) {
    var mm = new MotionMagicConfigs();
    mm.MotionMagicCruiseVelocity = cruiseVelocity;
    mm.MotionMagicAcceleration = acceleration;
    deployMotor.getConfigurator().apply(mm);
  }

  @Override
  public void updateInputs(CarriageIOInputs inputs) {
    inputs.deployPositionRotations = deployMotor.getPosition().getValueAsDouble();
    inputs.deployVelocityRotPerSec = deployMotor.getVelocity().getValueAsDouble();
    inputs.deployAppliedVolts      = deployMotor.getMotorVoltage().getValueAsDouble();
    inputs.deployCurrentAmps       = deployMotor.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public void setDeployPosition(double rotations) {
    deployMotor.setControl(deployRequest.withPosition(rotations));
  }
}
