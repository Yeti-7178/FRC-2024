package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    private final  CANSparkMax m_upperShooterMotorA = new CANSparkMax(ShooterConstants.kUpperShooterMotorPortA, MotorType.kBrushless);
    private final  CANSparkMax m_upperShooterMotorB = new CANSparkMax(ShooterConstants.kUpperShooterMotorPortB, MotorType.kBrushless);

    public ShooterSubsystem(){
        m_upperShooterMotorA.restoreFactoryDefaults();
        m_upperShooterMotorB.restoreFactoryDefaults();
        m_upperShooterMotorA.setSmartCurrentLimit(ShooterConstants.kIntakeCurrentLimit);
        m_upperShooterMotorA.setIdleMode(IdleMode.kBrake);
        m_upperShooterMotorB.setSmartCurrentLimit(ShooterConstants.kIntakeCurrentLimit);
        m_upperShooterMotorB.setIdleMode(IdleMode.kBrake);

    }

    public void shooterOn(){
        m_upperShooterMotorA.set(ShooterConstants.kUpperShooterMotorSpeedA); //ramp up?
        m_upperShooterMotorB.set(ShooterConstants.kUpperShooterMotorSpeedB); //ramp up?
    }
    public void setSlowSpeed()
    {
        m_upperShooterMotorA.set(.1); //ramp up?
        m_upperShooterMotorB.set(-.1); //ramp up?
    }
    public void setSpeed(double d)
    {
        m_upperShooterMotorA.set(d); //ramp up?
        m_upperShooterMotorB.set(-d); //ramp up?
    }

    public void shooterOff(){
        m_upperShooterMotorA.set(0); //fall off?
        m_upperShooterMotorB.set(0); //fall off?
    }

  
}
