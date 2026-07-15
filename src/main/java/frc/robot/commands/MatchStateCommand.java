// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MatchStateConfig;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Tracks the 2026 "Rebuilt" teleop shift schedule — Transition -> Shift1..4 -> Endgame — off a
 * wall clock started at {@link #initialize()}, and publishes the current phase / countdowns to
 * the "Elastic" NetworkTables table for the coach's dashboard (mirrors the in-field scoreboard's
 * shift display).
 *
 * <p>Phases: 0=Transition, 1-4=Shift1..Shift4, 5=Endgame. During Shift1..4 exactly one alliance's
 * HUB is active, alternating each shift; Transition and Endgame have both HUBs active (the shift
 * lockout is teleop-only). Which alliance is active first depends on which alliance scored more
 * Fuel in auto ("auto win") — the robot has no way to know that on its own, so it's a coach-set
 * toggle on the dashboard ({@code isAutoWinSupplier}), wired up in {@code RobotContainer}.
 *
 * <p>Scheduled once on the rising edge of teleop enabled (not autonomous — the shift schedule
 * doesn't start until teleop regardless, and gating on auto meant a driver who jumps straight to
 * the Teleoperated tab on the DS to practice never fired this at all). {@link #isFinished()} is
 * always false so it keeps counting through endgame instead of ending mid-phase.
 */
public class MatchStateCommand extends Command {

  // 0=transition, 1=shift1, 2=shift2, 3=shift3, 4=shift4, 5=endgame
  private static final int LAST_PHASE = 5; // endgame

  private double matchStartTime = 0.0;
  private int currentPhase = 0;
  private double phaseStartTime = 0.0;

  private final BooleanSupplier isAutoWinSupplier;

  private final StringPublisher matchTimePublisher;
  private final StringPublisher shiftTimePublisher;
  private final StringPublisher matchPhasePublisher;
  private final BooleanPublisher hubActivePublisher;

  public MatchStateCommand(BooleanSupplier isAutoWinSupplier) {
    this.isAutoWinSupplier = isAutoWinSupplier;

    NetworkTable table = NetworkTableInstance.getDefault().getTable("Elastic");
    matchTimePublisher = table.getStringTopic("MatchTime").publish();
    shiftTimePublisher = table.getStringTopic("ShiftTimeText").publish();
    matchPhasePublisher = table.getStringTopic("MatchPhase").publish();
    hubActivePublisher = table.getBooleanTopic("isHubActive").publish();
  }

  @Override
  public void initialize() {
    matchStartTime = Timer.getFPGATimestamp();
    currentPhase = 0;
    phaseStartTime = matchStartTime;
    updateAll();
  }

  @Override
  public void execute() {
    double now = Timer.getFPGATimestamp();

    double elapsedPhase = now - phaseStartTime;
    double duration = getPhaseDuration(currentPhase);

    if (elapsedPhase >= duration) {
      currentPhase++;
      if (currentPhase <= LAST_PHASE) {
        phaseStartTime = now;
      }
    }

    updateAll();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    matchTimePublisher.set("00:00");
    shiftTimePublisher.set("00:00");
    matchPhasePublisher.set("end");
    hubActivePublisher.set(false);
  }

  private void updateAll() {
    double now = Timer.getFPGATimestamp();

    double matchElapsed = now - matchStartTime;
    double matchRemaining = Math.max(0.0, getTotalTime() - matchElapsed);
    String matchTimeText = formatTime(matchRemaining);
    matchTimePublisher.set(matchTimeText);

    double shiftRemaining = getMergedRemainingTime(now);
    String shiftTimeText = formatTime(shiftRemaining);
    shiftTimePublisher.set(shiftTimeText);

    String phaseName = getPhaseName(currentPhase);
    matchPhasePublisher.set(phaseName);

    boolean hubActive = isPhaseActive(currentPhase);
    hubActivePublisher.set(hubActive);

    Logger.recordOutput("MatchState/MatchTime", matchTimeText);
    Logger.recordOutput("MatchState/ShiftTime", shiftTimeText);
    Logger.recordOutput("MatchState/Phase", phaseName);
    Logger.recordOutput("MatchState/HubActive", hubActive);
  }

  // Total teleop duration, derived from the phase durations so it can't drift out of sync with
  // them if the tunables get edited.
  private double getTotalTime() {
    return MatchStateConfig.TRANSITION_TIME_SEC.get()
        + 4 * MatchStateConfig.SHIFT_TIME_SEC.get()
        + MatchStateConfig.ENDGAME_TIME_SEC.get();
  }

  // Remaining time in the current phase, plus any immediately-following phases that share the
  // same HUB-active state (so the dashboard shows one continuous countdown for "how long until
  // this HUB goes inactive" rather than resetting every 25s).
  private double getMergedRemainingTime(double now) {
    double elapsed = now - phaseStartTime;
    double currentRemaining = Math.max(0.0, getPhaseDuration(currentPhase) - elapsed);

    double total = currentRemaining;

    if (isPhaseActive(currentPhase)) {
      int next = currentPhase + 1;
      while (next <= LAST_PHASE && isPhaseActive(next)) {
        total += getPhaseDuration(next);
        next++;
      }
    }

    return total;
  }

  private boolean isPhaseActive(int phase) {
    boolean isAutoWin = isAutoWinSupplier.getAsBoolean();

    switch (phase) {
      case 0: // transition
      case 5: // endgame
        return true;
      case 1:
        return !isAutoWin;
      case 2:
        return isAutoWin;
      case 3:
        return !isAutoWin;
      case 4:
        return isAutoWin;
      default:
        return false;
    }
  }

  private double getPhaseDuration(int phase) {
    switch (phase) {
      case 0:
        return MatchStateConfig.TRANSITION_TIME_SEC.get();
      case 1:
      case 2:
      case 3:
      case 4:
        return MatchStateConfig.SHIFT_TIME_SEC.get();
      case 5:
        return MatchStateConfig.ENDGAME_TIME_SEC.get();
      default:
        return 0.0;
    }
  }

  private String getPhaseName(int phase) {
    switch (phase) {
      case 0:
        return "TRANSITION";
      case 1:
        return "SHIFT1";
      case 2:
        return "SHIFT2";
      case 3:
        return "SHIFT3";
      case 4:
        return "SHIFT4";
      case 5:
        return "ENDGAME";
      default:
        return "END";
    }
  }

  private String formatTime(double time) {
    int totalSeconds = (int) Math.ceil(time);
    if (totalSeconds < 0) totalSeconds = 0;
    int minutes = totalSeconds / 60;
    int seconds = totalSeconds % 60;
    return String.format("%02d:%02d", minutes, seconds);
  }
}
