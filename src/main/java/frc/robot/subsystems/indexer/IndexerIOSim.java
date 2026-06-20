package frc.robot.subsystems.indexer;

import static frc.robot.Constants.IndexerConfig.*;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Runs the real TalonFX velocity loop inside the sim via getSimState (gains shared with real). */
public class IndexerIOSim extends IndexerIOTalonFX {

  private static final double DT = 0.020;

  private final DCMotorSim sim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(
          DCMotor.getKrakenX60Foc(1), INDEXER_MOI_KG_M2, INDEXER_GEAR_RATIO),
      DCMotor.getKrakenX60Foc(1));

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    TalonFXSimState s = leaderMotor.getSimState();
    s.setSupplyVoltage(12.0);
    sim.setInputVoltage(s.getMotorVoltage());
    sim.update(DT);
    s.setRawRotorPosition(sim.getAngularPositionRotations() * INDEXER_GEAR_RATIO);
    s.setRotorVelocity((sim.getAngularVelocityRPM() / 60.0) * INDEXER_GEAR_RATIO);

    // follower mirrors the leader; keep its sim supply voltage sane (not used for physics)
    followerMotor.getSimState().setSupplyVoltage(12.0);

    super.updateInputs(inputs);
  }
}
