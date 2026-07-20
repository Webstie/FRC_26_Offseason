package frc.robot.subsystems.carriage;

import static frc.robot.Constants.CarriageConfig.*;
import static frc.robot.Constants.SimConfig.*;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

/**
 * Simulation IO for the carriage deploy slide.
 *
 * <p>REUSES the real {@code MotionMagicVoltage} loop + Slot0 gains from {@link CarriageIOTalonFX} and
 * only adds the {@link ElevatorSim} plant plus Phoenix6 sim-state feedback, so sim and hardware share
 * one tuning. The closed loop runs inside the (simulated) TalonFX exactly as on the real robot.
 */
public class CarriageIOSim extends CarriageIOTalonFX {

  private static final double DT_SECONDS = 0.020;
  private static final double SUPPLY_VOLTAGE = 12.0; // fixed bus voltage (no battery-sag coupling)

  private final ElevatorSim deploySim = new ElevatorSim(
      DCMotor.getKrakenX60Foc(1),
      CARRIAGE_GEAR_RATIO,
      CARRIAGE_MASS_KG,
      CARRIAGE_DRUM_RADIUS_M,
      CARRIAGE_MIN_TRAVEL_M,
      CARRIAGE_MAX_TRAVEL_M,
      CARRIAGE_SIM_GRAVITY,
      0.0);

  @Override
  public void updateInputs(CarriageIOInputs inputs) {
    // Deploy: TalonFX rotor frame <-> ElevatorSim (meters).
    // deployMotor uses SensorToMechanismRatio = ROTOR_PER_METER, so raw rotor units = meters * ROTOR_PER_METER.
    TalonFXSimState deployState = deployMotor.getSimState();
    deployState.setSupplyVoltage(SUPPLY_VOLTAGE);
    deploySim.setInputVoltage(deployState.getMotorVoltage());
    deploySim.update(DT_SECONDS);
    deployState.setRawRotorPosition(deploySim.getPositionMeters() * ROTOR_PER_METER);
    deployState.setRotorVelocity(deploySim.getVelocityMetersPerSecond() * ROTOR_PER_METER);

    super.updateInputs(inputs);
  }
}
