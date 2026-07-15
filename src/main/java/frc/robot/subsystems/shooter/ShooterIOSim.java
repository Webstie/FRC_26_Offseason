package frc.robot.subsystems.shooter;

import static frc.robot.Constants.ShooterConfig.*;
import static frc.robot.Constants.SimConfig.*;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Runs the real TalonFX velocity loop inside the sim via getSimState (gains shared with real). */
public class ShooterIOSim extends ShooterIOTalonFX {

  private static final double DT = 0.020;
  private static final double SUPPLY_VOLTAGE = 12.0; // fixed bus voltage (no battery-sag coupling)

  private final DCMotorSim sim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(
          DCMotor.getKrakenX60Foc(1), SHOOTER_MOI_KG_M2, SHOOTER_GEAR_RATIO),
      DCMotor.getKrakenX60Foc(1));

  // Hood pivot plant. Like the real robot, the MotionMagic loop runs inside the (simulated) TalonFX;
  // we only feed it physics + CANcoder/rotor feedback. Starts at the rest angle.
  private final SingleJointedArmSim hoodSim = new SingleJointedArmSim(
      DCMotor.getKrakenX60Foc(1),
      HOOD_GEAR_RATIO,
      HOOD_MOI_KG_M2,
      HOOD_LENGTH_M,
      HOOD_MIN_ROTATIONS * 2.0 * Math.PI,
      HOOD_MAX_ROTATIONS * 2.0 * Math.PI,
      HOOD_SIM_GRAVITY,
      HOOD_REST_ROTATIONS.get() * 2.0 * Math.PI);

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    TalonFXSimState s = leftUpMotor.getSimState();
    // Match the leader's inverted (Clockwise_Positive) orientation so the sim sign convention agrees
    // with the real motor — otherwise the simulated velocity loop would read backwards.
    s.Orientation = ChassisReference.Clockwise_Positive;
    s.setSupplyVoltage(SUPPLY_VOLTAGE);
    sim.setInputVoltage(s.getMotorVoltage());
    sim.update(DT);
    s.setRawRotorPosition(sim.getAngularPositionRotations() * SHOOTER_GEAR_RATIO);
    s.setRotorVelocity((sim.getAngularVelocityRPM() / 60.0) * SHOOTER_GEAR_RATIO);

    // followers mirror the leader; keep their sim supply voltage sane (not used for physics)
    leftDownMotor.getSimState().setSupplyVoltage(SUPPLY_VOLTAGE);
    rightUpMotor.getSimState().setSupplyVoltage(SUPPLY_VOLTAGE);
    rightDownMotor.getSimState().setSupplyVoltage(SUPPLY_VOLTAGE);

    // --- Hood: TalonFX rotor + CANcoder frames <-> SingleJointedArmSim (radians) ---
    // The CANcoder reads hood rotations (1:1); the rotor spins HOOD_GEAR_RATIO faster.
    TalonFXSimState hs = hoodMotor.getSimState();
    CANcoderSimState cc = hoodCancoder.getSimState();
    hs.setSupplyVoltage(SUPPLY_VOLTAGE);
    cc.setSupplyVoltage(SUPPLY_VOLTAGE);
    hoodSim.setInputVoltage(hs.getMotorVoltage());
    hoodSim.update(DT);
    double hoodRot = hoodSim.getAngleRads() / (2.0 * Math.PI);
    double hoodRotPerSec = hoodSim.getVelocityRadPerSec() / (2.0 * Math.PI);
    cc.setRawPosition(hoodRot);
    cc.setVelocity(hoodRotPerSec);
    hs.setRawRotorPosition(hoodRot * HOOD_GEAR_RATIO);
    hs.setRotorVelocity(hoodRotPerSec * HOOD_GEAR_RATIO);

    super.updateInputs(inputs);
  }
}
