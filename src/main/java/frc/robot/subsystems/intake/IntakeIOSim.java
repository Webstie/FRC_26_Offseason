package frc.robot.subsystems.intake;

import static frc.robot.Constants.IntakeConfig.*;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

/**
 * Simulation IO for the intake.
 *
 * <p>This intentionally REUSES the real control stack from {@link IntakeIOTalonFX} — the same
 * {@code VelocityTorqueCurrentFOC} / {@code MotionMagicVoltage} requests and the same Slot0 gains —
 * and only adds the physics plant plus Phoenix6 sim-state feedback. The closed loop therefore runs
 * inside the (simulated) TalonFX exactly as it does on the real robot, so sim and hardware share one
 * tuning. There is no hand-rolled PID here.
 *
 * <p>Each loop we: feed the controller's decided motor voltage into the WPILib plant, step the
 * plant, then write the resulting ROTOR position/velocity back into the sim state so the TalonFX's
 * internal loop sees realistic sensor feedback. Note: like real hardware, the motor only produces
 * output while the robot is ENABLED — enable Teleop in the sim GUI to see the rollers spin.
 */
public class IntakeIOSim extends IntakeIOTalonFX {

  private static final double DT_SECONDS = 0.020;
  private static final double SUPPLY_VOLTAGE = 12.0; // fixed bus voltage (no battery-sag coupling)

  private final DCMotorSim rollerSim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), ROLLER_MOI_KG_M2, ROLLER_GEAR_RATIO),
      DCMotor.getKrakenX60Foc(1));

  private final ElevatorSim deploySim = new ElevatorSim(
      DCMotor.getKrakenX60Foc(1),
      DEPLOY_GEAR_RATIO,
      DEPLOY_MASS_KG,
      DEPLOY_DRUM_RADIUS_M,
      DEPLOY_MIN_TRAVEL_M,
      DEPLOY_MAX_TRAVEL_M,
      DEPLOY_SIM_GRAVITY,
      0.0);

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // --- Rollers: TalonFX rotor frame <-> DCMotorSim mechanism frame ---
    TalonFXSimState rollerState = leftMotor.getSimState();
    rollerState.setSupplyVoltage(SUPPLY_VOLTAGE);
    rollerSim.setInputVoltage(rollerState.getMotorVoltage());
    rollerSim.update(DT_SECONDS);
    // DCMotorSim reports the mechanism (post-reduction); the rotor spins ROLLER_GEAR_RATIO faster.
    rollerState.setRawRotorPosition(rollerSim.getAngularPositionRotations() * ROLLER_GEAR_RATIO);
    rollerState.setRotorVelocity((rollerSim.getAngularVelocityRPM() / 60.0) * ROLLER_GEAR_RATIO);

    // Follower mirrors the leader; keep its supply voltage sane (not used for physics).
    rightMotor.getSimState().setSupplyVoltage(SUPPLY_VOLTAGE);

    // --- Deploy: TalonFX rotor frame <-> ElevatorSim (meters) ---
    // deployMotor uses SensorToMechanismRatio = ROTOR_PER_METER, so raw rotor units = meters * ROTOR_PER_METER.
    TalonFXSimState deployState = deployMotor.getSimState();
    deployState.setSupplyVoltage(SUPPLY_VOLTAGE);
    deploySim.setInputVoltage(deployState.getMotorVoltage());
    deploySim.update(DT_SECONDS);
    deployState.setRawRotorPosition(deploySim.getPositionMeters() * ROTOR_PER_METER);
    deployState.setRotorVelocity(deploySim.getVelocityMetersPerSecond() * ROTOR_PER_METER);

    // Read the now sim-updated signals through the real IO path.
    super.updateInputs(inputs);
  }
}
