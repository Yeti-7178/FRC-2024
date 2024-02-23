package frc.robot.subsystems;

//command organization
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//motor stuff
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
//constants
import frc.robot.constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {
    private static final m_leftClimbMotor = new CANSparkMax(ClimbConstants.kLeftClimbCanId, MotorType.kBrushless);
    private static final m_rightClimbMotor = new CANSparkMax(ClimbConstants.kRightClimbCanId, MotorType.kBrushless);

    public ClimbSubsystem {
        m_leftClimbMotor.restoreFactoryDefaults();
        m_rightClimbMotor.restoreFactoryDefaults();
        m_leftClimbMotor.setIdleMode(ClimbConstants.kClimbIdleMode);
        m_rightClimbMotor.setIdleMode(ClimbConstants.kClimbIdleMode);
        m_leftClimbMotor.setInverted(ClimbConstants.kClimbLeftInverted);
        m_rightClimbMotor.setInverted(ClimbConstants.kClimbRightInverted);
        m_leftClimbMotor.burnFlash();
        m_rightClimbMotor.burnFlash();
    }

    //left motor
    public void climbLeftUp() {
        m_leftClimbMotor.set(ClimbConstants.kClimbMotorSpeed);
    }

    public void climbLeftDown() {
        m_leftClimbMotor.set(-ClimbConstants.kClimbMotorSpeed); 
    }

    public void climbLeftStop() {
        m_leftClimbMotor.stopMotor();
    }

    public void climbLeftSetSpeed(double speed) {
        m_leftClimbMotor.set(speed);
    }

    //right motor
    public void climbRightUp() {
        m_RightClimbMotor.set(ClimbConstants.kClimbMotorSpeed);
    }

    public void climbRightDown() {
        m_RightClimbMotor.set(-ClimbConstants.kClimbMotorSpeed); 
    }

    public void climbRightStop() {
        m_RightClimbMotor.stopMotor();
    }

    public void climbRightSetSpeed(double speed) {
        m_RightClimbMotor.set(speed);
    }

    //both motors
    public void climbUp() {
        climbRightUp();
        climbLeftUp();
    }

    public void climbDown() {
        climbLeftDown();
        climbRightDown();
    }

    public void climbStop() {
        climbLeftStop();
        climbRightStop();
    }

    public void climbSetSpeed(double speed) {
        climbLeftSetSpeed(speed);
        climbRightSetSpeed(speed);
    }
}
