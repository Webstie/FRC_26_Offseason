package frc.robot.subsystems.indexer;

import static frc.robot.Constants.IndexerConfig.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {

  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  public Indexer(IndexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);

    // Re-apply gains to the motor controller whenever any tunable changes.
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> io.setGains(INDEXER_KP.get(), INDEXER_KI.get(), INDEXER_KD.get(), INDEXER_KS.get(), INDEXER_KV.get()),
        INDEXER_KP, INDEXER_KI, INDEXER_KD, INDEXER_KS, INDEXER_KV);

    LoggedTunableNumber.ifChanged(
        hashCode() + 1,
        () -> io.setCurrentLimit(INDEXER_TORQUE_CURRENT_LIMIT.get()),
        INDEXER_TORQUE_CURRENT_LIMIT);

    LoggedTunableNumber.ifChanged(
        hashCode() + 2,
        () -> io.setSupplyCurrentLimit(INDEXER_SUPPLY_CURRENT_LIMIT.get()),
        INDEXER_SUPPLY_CURRENT_LIMIT);
  }

  public void setIndexerMotorVelocity(double rps) {
    io.setVelocity(rps);
  }
}
