// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  // CAN IDs 1-8 reserved for swerve drivetrain

  public static final class ShooterConfig{
    public static final int SHOOTER_LEFT_UP_MOTOR_ID    = 9;
    public static final int SHOOTER_LEFT_DOWN_MOTOR_ID  = 10;
    public static final int SHOOTER_RIGHT_UP_MOTOR_ID   = 11;
    public static final int SHOOTER_RIGHT_DOWN_MOTOR_ID = 12;

    public static final double SHOOTER_VELOCITY = 100.0;  // rotations per second
  }

  public static final class IntakeConfig{
    public static final int INTAKE_LEFT_MOTOR_ID    = 13;
    public static final int INTAKE_RIGHT_MOTOR_ID   = 14;
    public static final int INTAKE_DEPLOY_MOTOR_ID  = 15;

    public static final double INTAKE_VELOCITY = 60.0;            // rotations per second
    public static final double INTAKE_DEPLOY_DOWN_POSITION = 0.25; // rotations (deployed/open)
    public static final double INTAKE_DEPLOY_UP_POSITION   = 0.0;  // rotations (retracted/closed)
  }

  public static final class RollersConfig{
    public static final int ROLLERS_MOTOR_ID = 16;

    public static final double ROLLERS_VELOCITY = 50.0;  // rotations per second
  }

  public static final class IndexerConfig{
    public static final int INDEXER_LEADER_MOTOR_ID   = 17;
    public static final int INDEXER_FOLLOWER_MOTOR_ID = 18;

    public static final double INDEXER_VELOCITY = 50.0;  // rotations per second
  }
}
