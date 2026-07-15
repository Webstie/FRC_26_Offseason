// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

/** Teleop drive command factory. */
public final class DriveCommands {
  private static final double FF_START_DELAY = 2.0; // Seconds
  private static final double FF_RAMP_RATE = 0.1; // Volts per second

  private DriveCommands() {}

  /** Field-relative teleop drive: left stick translates, right stick rotates. */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          double maxV = drive.getMaxLinearSpeedMetersPerSec();
          double maxOmega = drive.getMaxAngularSpeedRadPerSec();
          double vx = deadband(xSupplier) * maxV;
          double vy = deadband(ySupplier) * maxV;
          double omega =
              deadband(omegaSupplier) * maxOmega * DriveConstants.TELEOP_ROTATION_SPEED_SCALAR;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, drive.getRotation()));
        },
        drive);
  }

  /** Deadbanded, sign-corrected stick value in [-1, 1] (caller scales by max speed). */
  private static double deadband(DoubleSupplier stick) {
    return -MathUtil.applyDeadband(stick.getAsDouble(), DriveConstants.JOYSTICK_DEADBAND);
  }

  /** Measures the velocity feedforward constants for the drive motors. */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new ArrayList<>();
    List<Double> voltageSamples = new ArrayList<>();
    Timer timer = new Timer();

    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  velocitySamples.clear();
                  voltageSamples.clear();
                  // Loud, impossible-to-miss marker that the command actually got scheduled. If
                  // you never see THIS line, the auto chooser isn't picking this command (check
                  // the "Auto Choices" widget is actually set to "Drive Simple FF
                  // Characterization" before enabling) -- the problem isn't console visibility,
                  // it's that the command never ran.
                  DriverStation.reportWarning(
                      "\n"
                          + "#".repeat(60)
                          + "\n"
                          + "###  DRIVE FF CHARACTERIZATION STARTED  ###\n"
                          + "#".repeat(60),
                      false);
                  SmartDashboard.putString("Drive/FFCharacterization/Status", "RUNNING");
                }),
            Commands.run(() -> drive.runCharacterization(0.0), drive).withTimeout(FF_START_DELAY),
            Commands.runOnce(timer::restart),
            Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive))
        .finallyDo(
            () -> {
              drive.stop();

              int n = velocitySamples.size();
              double sumX = 0.0;
              double sumY = 0.0;
              double sumXY = 0.0;
              double sumX2 = 0.0;
              for (int i = 0; i < n; i++) {
                sumX += velocitySamples.get(i);
                sumY += voltageSamples.get(i);
                sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
              }

              double denominator = n * sumX2 - sumX * sumX;
              if (n < 2 || Math.abs(denominator) < 1e-9) {
                String msg =
                    "\n"
                        + "*".repeat(60)
                        + "\n"
                        + "*** Drive FF Characterization FAILED: only "
                        + n
                        + " samples ***\n"
                        + "*".repeat(60);
                System.out.println(msg);
                DriverStation.reportError(msg, false);
                SmartDashboard.putString(
                    "Drive/FFCharacterization/Status", "FAILED (" + n + " samples)");
                return;
              }

              double kS = (sumY * sumX2 - sumX * sumXY) / denominator;
              double kV = (n * sumXY - sumX * sumY) / denominator;

              NumberFormat formatter = new DecimalFormat("#0.00000");
              String kSStr = formatter.format(kS);
              String kVStr = formatter.format(kV);
              String banner =
                  "\n\n"
                      + "*".repeat(60)
                      + "\n"
                      + "***** DRIVE FF CHARACTERIZATION RESULTS *****\n"
                      + "*****   kS: "
                      + kSStr
                      + "\n"
                      + "*****   kV: "
                      + kVStr
                      + "\n"
                      + "*****   (n="
                      + n
                      + " samples) -- copy into driveGains in TunerConstants.java\n"
                      + "*".repeat(60)
                      + "\n\n";
              // Printed AND reported as a DS warning AND pushed to the dashboard: three
              // independent places to find it, in case console scrollback/filters hide the other
              // two.
              System.out.println(banner);
              DriverStation.reportWarning(banner, false);
              SmartDashboard.putString("Drive/FFCharacterization/Status", "DONE");
              SmartDashboard.putString("Drive/FFCharacterization/kS", kSStr);
              SmartDashboard.putString("Drive/FFCharacterization/kV", kVStr);
            });
  }
}
