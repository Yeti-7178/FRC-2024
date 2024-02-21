package frc.robot.subsystems;

//constants
import frc.robot.Constants.LEDConstants;
//WPI Library
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class LEDSubsystem {
    private AddressableLED m_LED = new AddressableLED(LEDConstants.kLEDPort);
    private AddressableLEDBuffer m_LEDBuffer = new AddressableLEDBuffer(LEDConstants.kLEDLength);

    /* functions for establishing the LED, setting it in other files, starting/stopping */
    public void SetLEDLength(int length) { 
        //will probably need to add an AddressableLED parameter if multiple LED strips are used,
        //but I don't expect this to be the case--the docs say "Only one LED Driver is currently supported 
        //by the roboRIO"
        m_LED.setLength(length);
    }

    public void SetLED(AddressableLEDBuffer buffer) {
        //I honestly don't expect this to be called, the way I see it the LED will only be set in other
        //functions in here but we'll see
        m_LED.setData(buffer);
    }

    public void EnableLED() {
        m_LED.start();
    }

    public void DisableLED() {
        m_LED.stop();
    }

    /* functions for setting the LED color */
    //there's a lot of colors in the Color class, just look on the docs
    //does Color need to be imported?
    public void LEDGreen() {
        //iterate through the buffer and set each LED to green
        for (int i = 0; i < LEDConstants.kLEDLength; i++) {
            m_LEDBuffer.setLED(i, Color.kGreen);
        }

        //update the LEDs with the new buffer
        m_LED.setData(m_LEDBuffer);
    }

    public void LEDYeti() {
        //I'd like to make a gradient from yeti blue (which I found to be about 0, 115, 255 RGB) to white
        double redValue = 0;
        const double redValueIncrement = (255 - redValue) / LEDConstants.kLEDLength;
        double greenValue = 115;
        const double greenValueIncrement = (255 - greenValue) / LEDConstants.kLEDLength;

        //update the LED Buffer
        for (int i = 0; i < LEDConstants.kLEDLength; i++) {
            m_LEDBuffer.setLED(i, new Color(redValue, greenValue, 255)); //should I create a new color object every time or use something else?
            redValue += redValueIncrement;
            greenValue += greenValueIncrement;
        }

        //update the LEDs
        m_LED.setData(m_LEDBuffer);
    }
}
