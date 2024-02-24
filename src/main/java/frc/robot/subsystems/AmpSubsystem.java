//constants
import frc.robot.Constants.AmpConstants;
//for the code organization
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//for the sensor
import edu.wpi.first.wpilibj.DigitalInput;
//for the pneumatics
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Compressor;

public class AmpSubsystem extends SubsystemBase {
    //create the amp sensor and the pneumatic system for the amp
    //note for myself (Andy): the m_ prefix is for items that are members of the class
    private DigitalInput m_ampSensor = new DigitalInput(AmpConstants.kAmpSensorChannel);

    //note - PneumaticsModuleType.CTREPCM means the pneumatic module that the solenoid will connect to/be 
    //controlled from is a CTRE Pneumatics Control Module, not a REV Pneumatics Hub (REVPH).
    private DoubleSolenoid m_ampPusher = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, AmpConstants.kPusherForwardPort, AmpConstants.kPusherReversePort);
    private Compressor m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);


    //functions yippee
    /* sensor functions */
    public boolean getAmpIndex() {
        //return the state of the indexer sensor. Basically, return true if the ring is where it should be.
        return m_ampSensor.get();
    }

    /* solenoid functions */
    public void ampPusherOut() {
        //set m_ampPusher forward to push the note into the amp
        m_ampPusher.set(DoubleSolenoid.kForward);
    }

    public void ampPusherIn() {
        //set m_ampPusher reverse to retract the pusher into the robot
        m_ampPusher.set(DoubleSolenoid.kReverse);
    }

    public void ampToggle() {
        //toggle the amp's position. I don't expect this to be used unless we set a button for toggling the amp,
        //most likely for testing.
        m_ampPusher.toggle();
    }

    /* compressor functions */
    //enabling and disabling the compressor
    public void disableCompressor() {
        m_compressor.disable();
    }

    public void enableCompressor() {
        m_compressor.enableDigital();
    }

    public double getCompressorPressure() {
        //I don't know if this is supported by our compressor
        m_compressor.getPressure();
    }

}