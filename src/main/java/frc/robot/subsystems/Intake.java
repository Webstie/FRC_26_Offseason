package frc.robot.subsystems;
import static frc.robot.Constants.IntakeConfig.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{

        public final TalonFX intakeLeftMotor   = new TalonFX(INTAKE_LEFT_MOTOR_ID,   new CANBus("canivore"));
        public final TalonFX intakeRightMotor  = new TalonFX(INTAKE_RIGHT_MOTOR_ID,  new CANBus("canivore"));
        public final TalonFX intakeDeployMotor = new TalonFX(INTAKE_DEPLOY_MOTOR_ID, new CANBus("canivore"));

        private final VelocityTorqueCurrentFOC intakeLeftMotorRequest = new VelocityTorqueCurrentFOC(0.0).withSlot(0);
        private final MotionMagicVoltage intakeDeployMotorRequest = new MotionMagicVoltage(0.0).withSlot(0);

        public Intake() {

                var intakeMotorConfigs = new TalonFXConfiguration();
                intakeMotorConfigs.Slot0.kS = 0.0;
                intakeMotorConfigs.Slot0.kV = 0.0;
                intakeMotorConfigs.Slot0.kA = 0.0;
                intakeMotorConfigs.Slot0.kP = 0.0;
                intakeMotorConfigs.Slot0.kI = 0.0;
                intakeMotorConfigs.Slot0.kD = 0.0;
                intakeLeftMotor.getConfigurator().apply(intakeMotorConfigs);
                intakeRightMotor.getConfigurator().apply(intakeMotorConfigs);

                var intakeDeployMotorConfigs = new TalonFXConfiguration();
                intakeDeployMotorConfigs.Slot0.kS = 0.0;
                intakeDeployMotorConfigs.Slot0.kV = 0.0;
                intakeDeployMotorConfigs.Slot0.kA = 0.0;
                intakeDeployMotorConfigs.Slot0.kP = 0.0;
                intakeDeployMotorConfigs.Slot0.kI = 0.0;
                intakeDeployMotorConfigs.Slot0.kD = 0.0;
                intakeDeployMotorConfigs.MotionMagic.MotionMagicAcceleration = 0.0;
                intakeDeployMotorConfigs.MotionMagic.MotionMagicCruiseVelocity = 0.0;
                intakeDeployMotorConfigs.MotionMagic.MotionMagicExpo_kV = 0.0;
                intakeDeployMotorConfigs.MotionMagic.MotionMagicExpo_kA = 0.0;
                intakeDeployMotorConfigs.MotionMagic.MotionMagicJerk = 0;
                intakeDeployMotor.getConfigurator().apply(intakeDeployMotorConfigs);

                intakeRightMotor.setControl(new Follower(intakeLeftMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        }

        public void setIntakeMotorVelocity(double velocity) {
                intakeLeftMotor.setControl(intakeLeftMotorRequest.withVelocity(velocity));
        }

        public void setDeployMotorPosition(double position) {
                intakeDeployMotor.setControl(intakeDeployMotorRequest.withPosition(position));
        }
}
