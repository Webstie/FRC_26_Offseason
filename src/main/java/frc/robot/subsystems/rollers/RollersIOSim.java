package frc.robot.subsystems.rollers;

import static frc.robot.Constants.RollersConfig.*;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Runs the real TalonFX velocity loop inside the sim via getSimState (gains shared with real). */
public class RollersIOSim extends RollersIOTalonFX {

  private static final double DT = 0.020;

  private final DCMotorSim sim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(
          DCMotor.getKrakenX60Foc(1), ROLLERS_MOI_KG_M2, ROLLERS_GEAR_RATIO),
      DCMotor.getKrakenX60Foc(1));

  @Override
  public void updateInputs(RollersIOInputs inputs) {
    TalonFXSimState s = motor.getSimState();
    s.setSupplyVoltage(12.0);
    sim.setInputVoltage(s.getMotorVoltage());
    sim.update(DT);
    s.setRawRotorPosition(sim.getAngularPositionRotations() * ROLLERS_GEAR_RATIO);
    s.setRotorVelocity((sim.getAngularVelocityRPM() / 60.0) * ROLLERS_GEAR_RATIO);

    super.updateInputs(inputs);
  }
}
