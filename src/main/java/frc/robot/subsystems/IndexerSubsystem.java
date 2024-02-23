package frc.robot.subsystems;

//because it's a subsystem
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//constants
import frc.robot.Constants.IndexerConstants;
//stuff for motors
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;
//stuff for sensors
import edu.wpi.first.wpilibj.DigitalInput;


public class IndexerSubsystem extends SubsystemBase {
    private CANSparkMax m_indexConveyorMotor = new CanSparkMax(IndexerConstants.kIndexerConveyorMotorCanId, MotorType.kBrushed);
    private DigitalInput m_indexSensor = new DigitalInput(IndexerConstants.kIndexerSensorChannel);

    //constructor
    public IndexerSubsystem() {
        //motor init stuff. Noah put half of these commands in ty noah
        //ask about setting a smart current limit
        m_indexConveyorMotor.restoreFactoryDefaults();
        m_indexConveyorMotor.setIdleMode(IdleMode.kBrake);
        m_indexConveyorMotor.setInverted(IndexerConstants.kIndexerMotorInverted);
        m_indexConveyorMotor.burnFlash();
    }

    //motor functions
    public void setIndexConveyorSpeed(double speed) {
        m_indexConveyorMotor.set(speed);
    }

    public void stopIndexConveyor() {
        m_indexConveyorMotor.stopMotor();
    }

    public void runConveyorForward() {
        m_indexConveyorMotor.set(IndexerConstants.kIndexerConveyorStdSpeed);
    }

    public void runConveyorReverse() {
        m_indexConveyorMotor.set(-IndexerConstants.kIndexerConveyorStdSpeed);
    }

    //indexer functions
    public boolean getIndexerSensor() {
        //may need to negate this if it's a light sensor and returns true if nothing's detected
        return m_indexSensor.get();
    }
}
