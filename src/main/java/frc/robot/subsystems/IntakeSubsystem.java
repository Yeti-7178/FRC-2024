package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem  extends SubsystemBase{
    CANSparkMax m_intakeMotor = new CANSparkMax(IntakeConstants.KIntakeCanID, MotorType.kBrushless);

    public IntakeSubsystem()
    {
        m_intakeMotor.restoreFactoryDefaults();
        m_intakeMotor.setSmartCurrentLimit(IntakeConstants.kIntakeCurrentLimit);
        m_intakeMotor.setIdleMode(IdleMode.kBrake);
    }
    public void intakeRunForward()
    {
        m_intakeMotor.set(IntakeConstants.kIntakeMotorSpeed);
    }
    public void intakeRunBackwards()
    {
        m_intakeMotor.set(-IntakeConstants.kIntakeMotorSpeed);
    }
    public void intakeRunStop()
    {
        m_intakeMotor.set(0.0);
    }

    
}
