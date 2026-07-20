package frc.robot.subsystems.carriage;

import static frc.robot.Constants.CarriageConfig.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/** Carriage deploy slide: a linear mechanism that extends/retracts the intake. */
public class Carriage extends SubsystemBase {
  private final CarriageIO io;
  private final CarriageIOInputsAutoLogged inputs = new CarriageIOInputsAutoLogged();

  // Mechanism2d: simple side view of the carriage on the slide
  private final LoggedMechanism2d mechanism = new LoggedMechanism2d(1.0, 0.5);
  private final LoggedMechanismRoot2d mechRoot;
  private final LoggedMechanismLigament2d carriageLigament;

  private boolean deployed = false;

  public Carriage(CarriageIO io) {
    this.io = io;

    mechRoot = mechanism.getRoot("CarriageBase", 0.1, 0.1);
    carriageLigament = mechRoot.append(new LoggedMechanismLigament2d(
        "Carriage", 0.05, 0.0, 8.0, new Color8Bit(Color.kOrange)));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Carriage", inputs);

    // Re-apply gains to the deploy controller whenever any tunable changes.
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> io.setGains(
            CARRIAGE_KP.get(), CARRIAGE_KI.get(), CARRIAGE_KD.get(),
            CARRIAGE_KS.get(), CARRIAGE_KV.get(), CARRIAGE_KA.get()),
        CARRIAGE_KP, CARRIAGE_KI, CARRIAGE_KD, CARRIAGE_KS, CARRIAGE_KV, CARRIAGE_KA);

    // Re-apply the motion profile constraints whenever either tunable changes.
    LoggedTunableNumber.ifChanged(
        hashCode() + 1,
        () -> io.setMotionMagicConstraints(CARRIAGE_MM_CRUISE_MPS.get(), CARRIAGE_MM_ACCEL_MPS2.get()),
        CARRIAGE_MM_CRUISE_MPS, CARRIAGE_MM_ACCEL_MPS2);

    // Mechanism2d: ligament length grows with carriage travel
    carriageLigament.setLength(0.05 + inputs.deployPositionRotations);
    Logger.recordOutput("Carriage/Mechanism2d", mechanism);

    // Pose3d in robot frame for AdvantageScope 3D field
    Pose3d carriagePose = new Pose3d(
        CARRIAGE_BASE_TRANSLATION.plus(SLIDE_AXIS.times(inputs.deployPositionRotations)),
        new Rotation3d());
    Logger.recordOutput("Carriage/Pose3d", carriagePose);

    // Robot-relative component poses for the AdvantageScope custom model.
    // Order must match the model_N.glb files: [0] = sliding intake (model_0.glb).
    Logger.recordOutput("Carriage/ComponentPoses", carriagePose);
  }

  public void setDeployPosition(double rotations) {
    io.setDeployPosition(rotations);
  }

  /** Drive the carriage out to the deployed setpoint and mark it deployed. */
  public void deploy() {
    setDeployed(true);
  }

  /** Drive the carriage in to the retracted setpoint and mark it retracted. */
  public void retract() {
    setDeployed(false);
  }

  private void setDeployed(boolean deployed) {
    this.deployed = deployed;
    io.setDeployPosition(deployed ? CARRIAGE_DOWN_POSITION.get() : CARRIAGE_UP_POSITION.get());
  }

  /** True if the carriage was last commanded to the deployed (out) position. */
  public boolean isDeployed() {
    return deployed;
  }

  public Command deployCommand() {
    return runOnce(this::deploy);
  }

  public Command retractCommand() {
    return runOnce(this::retract);
  }

  public double getPositionRotations() {
    return inputs.deployPositionRotations;
  }
}
