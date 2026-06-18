package frc.robot.subsystems;
import static frc.robot.Constants.RollersConfig.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rollers extends SubsystemBase{

        public final TalonFX rollersMotor = new TalonFX(ROLLERS_MOTOR_ID, new CANBus("canivore"));
        private final VelocityTorqueCurrentFOC rollersMotorRequest = new VelocityTorqueCurrentFOC(0.0).withSlot(0);

        public Rollers() {
                
                var rollersMotorConfigs = new TalonFXConfiguration();
                rollersMotorConfigs.Slot0.kS = 0.0;
                rollersMotorConfigs.Slot0.kV = 0.0;
                rollersMotorConfigs.Slot0.kA = 0.0;
                rollersMotorConfigs.Slot0.kP = 0.0;
                rollersMotorConfigs.Slot0.kI = 0.0;
                rollersMotorConfigs.Slot0.kD = 0.0;
                rollersMotor.getConfigurator().apply(rollersMotorConfigs);
        }

            public void setRollersMotorVelocity(double velocity) {
                rollersMotor.setControl(rollersMotorRequest.withVelocity(velocity));
        }
}
