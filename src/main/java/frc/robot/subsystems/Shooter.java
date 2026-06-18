package frc.robot.subsystems;
import static frc.robot.Constants.ShooterConfig.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{

        public final TalonFX shooterLeftUpMotor    = new TalonFX(SHOOTER_LEFT_UP_MOTOR_ID,    new CANBus("canivore"));
        public final TalonFX shooterLeftDownMotor  = new TalonFX(SHOOTER_LEFT_DOWN_MOTOR_ID,  new CANBus("canivore"));
        public final TalonFX shooterRightUpMotor   = new TalonFX(SHOOTER_RIGHT_UP_MOTOR_ID,   new CANBus("canivore"));
        public final TalonFX shooterRightDownMotor = new TalonFX(SHOOTER_RIGHT_DOWN_MOTOR_ID, new CANBus("canivore"));

        private final VelocityTorqueCurrentFOC shooterLeftUpMotorRequest = new VelocityTorqueCurrentFOC(0.0).withSlot(0);

        public Shooter() {

                var shooterMotorConfigs = new TalonFXConfiguration();
                shooterMotorConfigs.Slot0.kS = 0.0;
                shooterMotorConfigs.Slot0.kV = 0.0;
                shooterMotorConfigs.Slot0.kA = 0.0;
                shooterMotorConfigs.Slot0.kP = 0.0;
                shooterMotorConfigs.Slot0.kI = 0.0;
                shooterMotorConfigs.Slot0.kD = 0.0;
                shooterLeftUpMotor.getConfigurator().apply(shooterMotorConfigs);
                shooterLeftDownMotor.getConfigurator().apply(shooterMotorConfigs);
                shooterRightUpMotor.getConfigurator().apply(shooterMotorConfigs);
                shooterRightDownMotor.getConfigurator().apply(shooterMotorConfigs);

                int leaderID = shooterLeftUpMotor.getDeviceID();
                shooterLeftDownMotor.setControl(new Follower(leaderID, MotorAlignmentValue.Aligned));
                shooterRightUpMotor.setControl(new Follower(leaderID, MotorAlignmentValue.Opposed));
                shooterRightDownMotor.setControl(new Follower(leaderID, MotorAlignmentValue.Opposed));
        }

            public void setShooterMotorVelocity(double velocity) {
                shooterLeftUpMotor.setControl(shooterLeftUpMotorRequest.withVelocity(velocity));
        }
}
