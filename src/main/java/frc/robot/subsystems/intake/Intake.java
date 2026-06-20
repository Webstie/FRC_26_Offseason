package frc.robot.subsystems.intake;

import static frc.robot.Constants.IntakeConfig.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Intake extends SubsystemBase {
  public boolean intakePitchPositionFlag = false;

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  // Mechanism2d: simple side view of the carriage on the slide
  private final LoggedMechanism2d mechanism = new LoggedMechanism2d(1.0, 0.5);
  private final LoggedMechanismRoot2d mechRoot;
  private final LoggedMechanismLigament2d carriageLigament;

  public Intake(IntakeIO io) {
    this.io = io;

    mechRoot = mechanism.getRoot("IntakeBase", 0.1, 0.1);
    carriageLigament = mechRoot.append(new LoggedMechanismLigament2d(
        "Carriage", 0.05, 0.0, 8.0, new Color8Bit(Color.kOrange)));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    // Mechanism2d: ligament length grows with carriage travel
    carriageLigament.setLength(0.05 + inputs.deployPositionMeters);
    Logger.recordOutput("Intake/Mechanism2d", mechanism);

    // Pose3d in robot frame for AdvantageScope 3D field
    Pose3d intakePose = new Pose3d(
        INTAKE_BASE_TRANSLATION.plus(SLIDE_AXIS.times(inputs.deployPositionMeters)),
        new Rotation3d());
    Logger.recordOutput("Intake/Pose3d", intakePose);

    // Robot-relative component poses for the AdvantageScope custom model.
    // Order must match the model_N.glb files: [0] = sliding carriage, [1] = spinning roller.
    Pose3d carriagePose = intakePose; // translates along the slide with deploy
    double rollerAngleRad = inputs.rollerPositionRot / ROLLER_GEAR_RATIO * 2.0 * Math.PI;
    Pose3d rollerPose = new Pose3d(
        ROLLER_VIS_TRANSLATION.plus(SLIDE_AXIS.times(inputs.deployPositionMeters)),
        new Rotation3d(0.0, rollerAngleRad, 0.0)); // spins about Y; moves out with the carriage
    Logger.recordOutput("Intake/ComponentPoses", carriagePose, rollerPose);
  }

  public void setIntakeMotorVelocity(double rps) {
    io.setRollerVelocity(rps);
  }

  public void setDeployMotorPosition(double meters) {
    io.setDeployPosition(meters);
  }

  public Command changePitchPositionCommand() {
    return runOnce(() -> intakePitchPositionFlag = !intakePitchPositionFlag);
  }

  public boolean getIntakePitchFlag() {
    return intakePitchPositionFlag;
  }
}
