package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    private final  CANSparkMax m_upperShooterMotorA = new CANSparkMax(IntakeConstants.KIntakeCanID, MotorType.kBrushless);
    private final  CANSparkMax m_upperShooterMotorB = new CANSparkMax(IntakeConstants.KIntakeCanID, MotorType.kBrushless);

    public ShooterSubsystem(){
        m_upperShooterMotorA.restoreFactoryDefaults();
        m_upperShooterMotorB.restoreFactoryDefaults();
        m_upperShooterMotorA.setSmartCurrentLimit(IntakeConstants.kIntakeCurrentLimit);
        m_upperShooterMotorA.setIdleMode(IdleMode.kBrake);
        m_upperShooterMotorB.setSmartCurrentLimit(IntakeConstants.kIntakeCurrentLimit);
        m_upperShooterMotorB.setIdleMode(IdleMode.kBrake);

    }

    public void shooterOn(){
        m_upperShooterMotorA.set(ShooterConstants.kUpperShooterMotorSpeedA); //ramp up?
        m_upperShooterMotorB.set(ShooterConstants.kUpperShooterMotorSpeedB); //ramp up?
    }

    public void shooterOff(){
        m_upperShooterMotorA.set(0); //fall off?
        m_upperShooterMotorB.set(0); //fall off?
    }
}
