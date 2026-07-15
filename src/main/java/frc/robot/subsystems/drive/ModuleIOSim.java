package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

/**
 * Physics-sim module IO backed by a maple-sim {@link SwerveModuleSimulation}. The maple-sim physics
 * world (advanced by {@code SimulatedArena.simulationPeriodic()} in {@link Drive}) integrates the
 * dynamics; this class just runs a simple voltage closed loop and reads the module state back.
 *
 * <p>Sim uses simple PID/FF (not the TunerConstants gains, which are tuned for real torque/voltage).
 */
public class ModuleIOSim implements ModuleIO {
  private static final double DRIVE_KP = 0.05;
  private static final double DRIVE_KD = 0.0;
  private static final double DRIVE_KS = 0.0;
  private static final double DRIVE_KV_ROT = 0.91035; // (volt * sec) / rotation
  private static final double DRIVE_KV = 1.0 / Units.rotationsToRadians(1.0 / DRIVE_KV_ROT);
  private static final double TURN_KP = 8.0;
  // P-only steer (the stock template default) limit-cycles against maple-sim's azimuth inertia +
  // friction, causing a visible at-rest jitter. A little derivative damps it; keep it well below
  // ~0.5 (which re-introduces oscillation).
  private static final double TURN_KD = 0.1;
  // Parked-stop: once the steer is on-target and at rest, stop commanding it so the loop can't hunt.
  private static final double TURN_PARKED_TOLERANCE_RAD = Units.degreesToRadians(0.5);
  private static final double TURN_PARKED_SPEED_RAD_PER_SEC = Units.degreesToRadians(2.0);

  private final SwerveModuleSimulation moduleSimulation;
  private final SimulatedMotorController.GenericMotorController driveMotor;
  private final SimulatedMotorController.GenericMotorController turnMotor;

  private boolean driveClosedLoop = false;
  private boolean turnClosedLoop = false;
  private final PIDController driveController = new PIDController(DRIVE_KP, 0, DRIVE_KD);
  private final PIDController turnController = new PIDController(TURN_KP, 0, TURN_KD);
  private double driveFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public ModuleIOSim(SwerveModuleSimulation moduleSimulation) {
    this.moduleSimulation = moduleSimulation;
    this.driveMotor =
        moduleSimulation.useGenericMotorControllerForDrive().withCurrentLimit(Amps.of(120.0));
    this.turnMotor =
        moduleSimulation.useGenericControllerForSteer().withCurrentLimit(Amps.of(20.0));

    // Enable wrapping for turn PID
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Run closed-loop control
    if (driveClosedLoop) {
      driveAppliedVolts =
          driveFFVolts
              + driveController.calculate(
                  moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond));
    } else {
      driveController.reset();
    }
    if (turnClosedLoop) {
      double measuredRad = moduleSimulation.getSteerAbsoluteFacing().getRadians();
      double errorRad = MathUtil.angleModulus(turnController.getSetpoint() - measuredRad);
      double steerSpeedRadPerSec =
          moduleSimulation.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);
      // Parked on-target: hold 0 V so the P loop can't limit-cycle (the at-rest jitter). The windows
      // are far tighter than any real steer target, so this never interferes with reorientation.
      if (Math.abs(errorRad) < TURN_PARKED_TOLERANCE_RAD
          && Math.abs(steerSpeedRadPerSec) < TURN_PARKED_SPEED_RAD_PER_SEC) {
        turnAppliedVolts = 0.0;
        turnController.reset();
      } else {
        turnAppliedVolts = turnController.calculate(measuredRad);
      }
    } else {
      turnController.reset();
    }

    // Feed requested voltages into the maple-sim physics
    driveMotor.requestVoltage(Volts.of(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0)));
    turnMotor.requestVoltage(Volts.of(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0)));

    // Update drive inputs
    inputs.driveConnected = true;
    inputs.drivePositionRad = moduleSimulation.getDriveWheelFinalPosition().in(Radians);
    inputs.driveVelocityRadPerSec = moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond);
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = Math.abs(moduleSimulation.getDriveMotorStatorCurrent().in(Amps));

    // Update turn inputs
    inputs.turnConnected = true;
    inputs.turnEncoderConnected = true;
    inputs.turnAbsolutePosition = moduleSimulation.getSteerAbsoluteFacing();
    inputs.turnPosition = moduleSimulation.getSteerAbsoluteFacing();
    inputs.turnVelocityRadPerSec =
        moduleSimulation.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = Math.abs(moduleSimulation.getSteerMotorStatorCurrent().in(Amps));

    // Single 50 Hz odometry sample (high-frequency odometry doesn't matter in sim)
    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveClosedLoop = false;
    driveAppliedVolts = output;
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnClosedLoop = false;
    turnAppliedVolts = output;
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    driveClosedLoop = true;
    driveFFVolts = DRIVE_KS * Math.signum(velocityRadPerSec) + DRIVE_KV * velocityRadPerSec;
    driveController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnClosedLoop = true;
    turnController.setSetpoint(rotation.getRadians());
  }
}
