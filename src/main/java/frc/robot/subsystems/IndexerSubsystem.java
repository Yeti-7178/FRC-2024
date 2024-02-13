package frc.robot.subsystems;

//because it's a subsystem
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//constants
import frc.robot.Constants.IndexerConstants;
//stuff for motors
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
//stuff for sensors
import edu.wpi.first.wpilibj.DigitalInput;


public class IndexerSubsystem extends SubsystemBase {
    private CANSparkMax m_indexConveyorMotor = new CanSparkMax(IndexerConstants.kIndexerConveyorMotorPort, MotorType.kBrushless);
    private DigitalInput m_indexSensor = new DigitalInput(IndexerConstants.kIndexerSensorChannel);
}
