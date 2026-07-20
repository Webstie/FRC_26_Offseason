// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  // Logged every loop so a real-match voltage sag / brownout reset shows up in the log with a
  // timestamp instead of having to guess from DS "lost communication" reports after the fact.
  private final Alert brownoutAlert =
      new Alert("Brownout: battery voltage sagged enough to trip roboRIO protection.", AlertType.kError);

  public Robot() {
    Logger.recordMetadata("ProjectName", "26Offseason");
    Logger.recordMetadata("RobotMode", isReal() ? "REAL" : "SIM");

    if (isReal()) {
      // Explicit path to internal storage -- the no-arg constructor auto-detects a USB stick at
      // /U/logs/ and silently fails (logged as an error on a background thread, no crash) if
      // there isn't one mounted. Point it at internal storage so logging always actually works.
      Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs/"));
      Logger.addDataReceiver(new NT4Publisher());
    } else {
      Logger.addDataReceiver(new NT4Publisher());
    }

    Logger.start();

    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    boolean brownedOut = RobotController.isBrownedOut();
    brownoutAlert.set(brownedOut);
    Logger.recordOutput("Power/BatteryVoltageV", RobotController.getBatteryVoltage());
    Logger.recordOutput("Power/BrownedOut", brownedOut);
    Logger.recordOutput("Power/InputCurrentA", RobotController.getInputCurrent());
    Logger.recordOutput("Power/CANUtilizationPct", RobotController.getCANStatus().percentBusUtilization);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
