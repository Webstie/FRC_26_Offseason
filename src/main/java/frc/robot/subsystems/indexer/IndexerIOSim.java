package frc.robot.subsystems.indexer;

import static frc.robot.Constants.SimConfig.*;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Runs the real TalonFX velocity loop inside the sim via getSimState (gains shared with real). */
public class IndexerIOSim extends IndexerIOTalonFX {

  private static final double DT = 0.020;
  private static final double SUPPLY_VOLTAGE = 12.0; // fixed bus voltage (no battery-sag coupling)

  private final DCMotorSim sim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(
          DCMotor.getKrakenX60Foc(1), INDEXER_MOI_KG_M2, INDEXER_GEAR_RATIO),
      DCMotor.getKrakenX60Foc(1));

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    TalonFXSimState leaderState   = leaderMotor.getSimState();
    TalonFXSimState followerState = followerMotor.getSimState();
    leaderState.setSupplyVoltage(SUPPLY_VOLTAGE);
    followerState.setSupplyVoltage(SUPPLY_VOLTAGE);
    // Single one-motor plant driven by the leader's output; the follower mirrors the leader, so
    // both sim states are fed the same rotor state.
    sim.setInputVoltage(leaderState.getMotorVoltage());
    sim.update(DT);
    double rotorRotations = sim.getAngularPositionRotations() * INDEXER_GEAR_RATIO;
    double rotorRPS = (sim.getAngularVelocityRPM() / 60.0) * INDEXER_GEAR_RATIO;
    leaderState.setRawRotorPosition(rotorRotations);
    leaderState.setRotorVelocity(rotorRPS);
    followerState.setRawRotorPosition(rotorRotations);
    followerState.setRotorVelocity(rotorRPS);

    super.updateInputs(inputs);
  }
}
