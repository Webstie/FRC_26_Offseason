package frc.robot.subsystems.shooter;

import static frc.robot.Constants.ShooterConfig.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    // Re-apply gains to the flywheel controller whenever any tunable changes.
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> io.setGains(SHOOTER_KP.get(), SHOOTER_KI.get(), SHOOTER_KD.get(), SHOOTER_KS.get(), SHOOTER_KV.get()),
        SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, SHOOTER_KS, SHOOTER_KV);

    LoggedTunableNumber.ifChanged(
        hashCode() + 1,
        () -> io.setCurrentLimit(SHOOTER_TORQUE_CURRENT_LIMIT.get()),
        SHOOTER_TORQUE_CURRENT_LIMIT);

    // Re-apply gains to the hood controller whenever any tunable changes.
    LoggedTunableNumber.ifChanged(
        hashCode() + 2,
        () -> io.setHoodGains(
            HOOD_KP.get(), HOOD_KI.get(), HOOD_KD.get(), HOOD_KS.get(), HOOD_KV.get(), HOOD_KA.get()),
        HOOD_KP, HOOD_KI, HOOD_KD, HOOD_KS, HOOD_KV, HOOD_KA);

    LoggedTunableNumber.ifChanged(
        hashCode() + 3,
        () -> io.setHoodMotionMagicConstraints(HOOD_MM_CRUISE_RPS.get(), HOOD_MM_ACCEL_RPS2.get()),
        HOOD_MM_CRUISE_RPS, HOOD_MM_ACCEL_RPS2);
  }

  public void setShooterMotorVelocity(double rps) {
    io.setVelocity(rps);
  }

  /** Coast the flywheel: it spins down on its own inertia instead of the motors braking it. */
  public void stopShooter() {
    io.stop();
  }

  /** Command the hood pivot to a position (mechanism rotations). */
  public void setHoodPosition(double rotations) {
    io.setHoodPosition(rotations);
  }

  public double getVelocityRPS() {
    return inputs.velocityRPS;
  }

  public double getHoodPositionRotations() {
    return inputs.hoodPositionRotations;
  }

  /** True once the flywheel is within tolerance of {@code targetRPS}. */
  public boolean atVelocity(double targetRPS) {
    return Math.abs(inputs.velocityRPS - targetRPS) <= SHOOTER_VELOCITY_TOLERANCE_RPS.get();
  }

  /** True once the hood is within tolerance of {@code targetRotations}. */
  public boolean atHoodPosition(double targetRotations) {
    return Math.abs((inputs.hoodPositionRotations) - targetRotations) <= HOOD_TOLERANCE_ROTATIONS.get();
  }
}
