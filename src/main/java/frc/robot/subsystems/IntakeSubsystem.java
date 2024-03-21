package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem  extends SubsystemBase{
    CANSparkMax m_intakeMotor = new CANSparkMax(IntakeConstants.KIntakeCanID, MotorType.kBrushed);
    private DigitalInput m_intakeSensor = new DigitalInput(6);
    

    public IntakeSubsystem()
    {
        m_intakeMotor.restoreFactoryDefaults();
        m_intakeMotor.setSmartCurrentLimit(IntakeConstants.kIntakeCurrentLimit);
        m_intakeMotor.setIdleMode(IdleMode.kBrake);
        Shuffleboard.getTab("Sensor").add("test", getIntakeSensor()).getEntry();
    }
    public void intakeRunForward()
    {
        m_intakeMotor.set(IntakeConstants.kIntakeMotorSpeed);
    }
    public void intakeRunBackwards()
    {
        m_intakeMotor.set(-IntakeConstants.kIntakeMotorSpeed);
    }
    public void intakeStop()
    {
        m_intakeMotor.set(0.0);
    }
    public boolean getIntakeSensor() {
        //may need to negate this if it's a light sensor and returns true if nothing's detected
        return m_intakeSensor.get();
    }

    @Override
    public void periodic()
    {
       
    }

    
}
