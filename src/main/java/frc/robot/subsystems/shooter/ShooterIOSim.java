package frc.robot.subsystems.shooter;

import static frc.robot.Constants.ShooterConfig.*;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Runs the real TalonFX velocity loop inside the sim via getSimState (gains shared with real). */
public class ShooterIOSim extends ShooterIOTalonFX {

  private static final double DT = 0.020;

  private final DCMotorSim sim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(
          DCMotor.getKrakenX60Foc(1), SHOOTER_MOI_KG_M2, SHOOTER_GEAR_RATIO),
      DCMotor.getKrakenX60Foc(1));

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    TalonFXSimState s = leftUpMotor.getSimState();
    s.setSupplyVoltage(12.0);
    sim.setInputVoltage(s.getMotorVoltage());
    sim.update(DT);
    s.setRawRotorPosition(sim.getAngularPositionRotations() * SHOOTER_GEAR_RATIO);
    s.setRotorVelocity((sim.getAngularVelocityRPM() / 60.0) * SHOOTER_GEAR_RATIO);

    // followers mirror the leader; keep their sim supply voltage sane (not used for physics)
    leftDownMotor.getSimState().setSupplyVoltage(12.0);
    rightUpMotor.getSimState().setSupplyVoltage(12.0);
    rightDownMotor.getSimState().setSupplyVoltage(12.0);

    super.updateInputs(inputs);
  }
}
