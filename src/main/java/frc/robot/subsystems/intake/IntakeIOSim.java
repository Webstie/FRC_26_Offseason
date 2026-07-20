package frc.robot.subsystems.intake;

import static frc.robot.Constants.SimConfig.*;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Simulation IO for the intake rollers.
 *
 * <p>This intentionally REUSES the real control stack from {@link IntakeIOTalonFX} — the same
 * {@code VelocityTorqueCurrentFOC} request and Slot0 gains — and only adds the physics plant plus
 * Phoenix6 sim-state feedback, so sim and hardware share one tuning. There is no hand-rolled PID.
 *
 * <p>Each loop we feed the controller's decided motor voltage into the WPILib plant, step it, then
 * write the resulting ROTOR position/velocity back into the sim state. Like real hardware, the motor
 * only produces output while the robot is ENABLED.
 */
public class IntakeIOSim extends IntakeIOTalonFX {

  private static final double DT_SECONDS = 0.020;
  private static final double SUPPLY_VOLTAGE = 12.0; // fixed bus voltage (no battery-sag coupling)

  private final DCMotorSim rollerSim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), INTAKE_MOI_KG_M2, INTAKE_GEAR_RATIO),
      DCMotor.getKrakenX60Foc(1));

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Rollers: TalonFX rotor frame <-> DCMotorSim mechanism frame
    TalonFXSimState rollerState = leftMotor.getSimState();
    rollerState.setSupplyVoltage(SUPPLY_VOLTAGE);
    rollerSim.setInputVoltage(rollerState.getMotorVoltage());
    rollerSim.update(DT_SECONDS);
    // DCMotorSim reports the mechanism (post-reduction); the rotor spins INTAKE_GEAR_RATIO faster.
    rollerState.setRawRotorPosition(rollerSim.getAngularPositionRotations() * INTAKE_GEAR_RATIO);
    rollerState.setRotorVelocity((rollerSim.getAngularVelocityRPM() / 60.0) * INTAKE_GEAR_RATIO);

    // Follower mirrors the leader; keep its supply voltage sane (not used for physics).
    rightMotor.getSimState().setSupplyVoltage(SUPPLY_VOLTAGE);

    super.updateInputs(inputs);
  }
}
