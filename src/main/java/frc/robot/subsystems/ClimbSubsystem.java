package frc.robot.subsystems;

//command organization
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//motor stuff
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
//limit switches
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

//constants
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {
    //motors
    private static final CANSparkMax m_leftClimbMotor = new CANSparkMax(ClimbConstants.kLeftClimbCanId, MotorType.kBrushless);
    private static final CANSparkMax m_rightClimbMotor = new CANSparkMax(ClimbConstants.kRightClimbCanId, MotorType.kBrushless);
    //sensors
    private static final DigitalInput m_topRightLimit = new DigitalInput(ClimbConstants.kTopRightLimitChannel);
    private static final DigitalInput m_bottomRightLimit = new DigitalInput(ClimbConstants.kBottomRightLimitChannel);
    private static final DigitalInput m_topLeftLimit = new DigitalInput(ClimbConstants.kTopLeftLimitChannel);
    private static final DigitalInput m_bottomLeftLimit = new DigitalInput(ClimbConstants.kBottomLeftLimitChannel);
    //shuffleboard
    private final ShuffleboardTab climbShuffleboard = Shuffleboard.getTab("climb");

    public ClimbSubsystem() {
        m_leftClimbMotor.restoreFactoryDefaults();
        m_rightClimbMotor.restoreFactoryDefaults();
        m_leftClimbMotor.setIdleMode(ClimbConstants.kClimbIdleMode);
        m_rightClimbMotor.setIdleMode(ClimbConstants.kClimbIdleMode);
        m_leftClimbMotor.setInverted(ClimbConstants.kClimbLeftInverted);
        m_rightClimbMotor.setInverted(ClimbConstants.kClimbRightInverted);
        m_leftClimbMotor.burnFlash();
        m_rightClimbMotor.burnFlash();

        climbShuffleboard.addBoolean("Top right", () -> getTopRightLimit());
        climbShuffleboard.addBoolean("Bottom right", () -> getBottomRightLimit());
        climbShuffleboard.addBoolean("Top left", () -> getTopLeftLimit());
        climbShuffleboard.addBoolean("Bottom left", () -> getBottomLeftLimit());
        climbShuffleboard.addDouble("Left motor", () -> m_leftClimbMotor.get());
        climbShuffleboard.addDouble("Right motor", () -> m_rightClimbMotor.get());
    }

    //left motor
    public void climbLeftSetSpeed(double speed) {
        m_leftClimbMotor.set(speed);
        //try applying deadbands
        /*if((m_bottomLeftLimit.get() && m_leftClimbMotor.get() < 0) || (!m_topLeftLimit.get() && m_leftClimbMotor.get() > 0)) {
            m_leftClimbMotor.set(0);
        } else {
            m_leftClimbMotor.set(speed);
        }*/
    }

    public void climbLeftUp() {
        if (m_topLeftLimit.get()) {
            //all of the limits output a 0 when the magnet's there except for top left
            climbLeftSetSpeed(ClimbConstants.kClimbMotorSpeed);
        } else climbLeftStop();
    }

    public void climbLeftDown() {
        if (!m_bottomLeftLimit.get()) {
            climbLeftSetSpeed(-ClimbConstants.kClimbMotorSpeed); 
        } else climbLeftStop();
    }

    public void climbLeftStop() {
        m_leftClimbMotor.stopMotor();
    }

    //right motor
    private void climbRightSetSpeed(double speed) {
        m_rightClimbMotor.set(speed);
        /*if((m_bottomRightLimit.get() && m_rightClimbMotor.get() < 0) || (m_topRightLimit.get() && m_rightClimbMotor.get() > 0)) {
            m_rightClimbMotor.set(0);
        } else {
            m_rightClimbMotor.set(speed);
        }*/
    }

    public void climbRightUp() {
        if (!m_topRightLimit.get()) {
            climbRightSetSpeed(ClimbConstants.kClimbMotorSpeed);
        } else climbRightStop();
    }

    public void climbRightDown() {
        if (!m_bottomRightLimit.get()) {
            climbRightSetSpeed(-ClimbConstants.kClimbMotorSpeed); 
        } else climbRightStop();
    }

    public void climbRightStop() {
        m_rightClimbMotor.stopMotor();
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

    //limit switches
    public boolean getTopRightLimit() {
        return m_topRightLimit.get();
    }

    public boolean getBottomRightLimit() {
        return m_bottomRightLimit.get();
    }

    public boolean getTopLeftLimit() {
        return m_topLeftLimit.get();
    }

    public boolean getBottomLeftLimit() 
    {
        return m_bottomLeftLimit.get();
    }

    
//     @Override
//     public void periodic() {
//          Shuffleboard.getTab("Sensor").add("Limit 1", getLimit1());
//         Shuffleboard.getTab("Sensor").add("Limit 2", getLimit2());
//         Shuffleboard.getTab("Sensor").add("Limit 3", getLimit3());
//         Shuffleboard.getTab("Sensor").add("Limit 4",getLimit4());
//     }
}
