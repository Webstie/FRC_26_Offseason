package frc.robot.subsystems;
import static frc.robot.Constants.IndexerConfig.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase{

        public final TalonFX indexerLeaderMotor = new TalonFX(INDEXER_LEADER_MOTOR_ID, new CANBus("canivore"));
        public final TalonFX indexerFollowerMotor = new TalonFX(INDEXER_FOLLOWER_MOTOR_ID, new CANBus("canivore"));
        private final VelocityTorqueCurrentFOC indexerLeaderMotorRequest = new VelocityTorqueCurrentFOC(0.0).withSlot(0);

        public Indexer() {

                var indexerMotorConfigs = new TalonFXConfiguration();
                indexerMotorConfigs.Slot0.kS = 0.0;
                indexerMotorConfigs.Slot0.kV = 0.0;
                indexerMotorConfigs.Slot0.kA = 0.0;
                indexerMotorConfigs.Slot0.kP = 0.0;
                indexerMotorConfigs.Slot0.kI = 0.0;
                indexerMotorConfigs.Slot0.kD = 0.0;
                indexerLeaderMotor.getConfigurator().apply(indexerMotorConfigs);
                indexerFollowerMotor.getConfigurator().apply(indexerMotorConfigs);

                indexerFollowerMotor.setControl(new Follower(indexerLeaderMotor.getDeviceID(), MotorAlignmentValue.Aligned));
        }

            public void setIndexerMotorVelocity(double velocity) {
                indexerLeaderMotor.setControl(indexerLeaderMotorRequest.withVelocity(velocity));
        }
}
