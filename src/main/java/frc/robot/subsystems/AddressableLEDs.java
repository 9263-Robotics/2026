package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

/**
 * A simple subsystem to control an addressable LED strip.
 * The strip needs to be set up in 1 chain, from start to end. Y splits will cause both strips to show the same pattern.
 * Should be plugged into the pwm 0 port on the roboRIO, and the ground and power on the PDH.
 */
public class AddressableLEDs extends SubsystemBase {

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(kLedStripLength);;

  // PWM port for the LED strip. This must be a PWM header on the roboRIO.
  private static final int kLedPort = 0;

  // Number of LEDs on the strip.
  private static final int kLedStripLength = 29; //183 for bunper strip last year TODO: update to actual length. 29 is one small strip

  AddressableLEDBufferView m_LedSection1 = m_ledBuffer.createView(0, 10); // Sections of controllable LEDs, can be used for different patterns/effects -
  AddressableLEDBufferView m_LedSection2 = m_ledBuffer.createView(11, 28); // This should probably be split where the strips are split and conected with wires.
  //AddressableLEDBufferView m_LedSection3 = m_ledBuffer.createView(90, 163); // Also sections may need to be reversed to adjust for wiring direction
  //AddressableLEDBufferView m_LedSection4 = m_ledBuffer.createView(164, 182);

  //set up patterns

  public static LEDPattern m_off = LEDPattern.kOff;

  //Variables for patterns

  private boolean flashState = false;
  private double lastFlashTime = 0;
  private static final double FLASH_INTERVAL = 0.05; // seconds between flashes


  public enum PatternMode {
    FLASHBANG,
    OFF
  }  
  

  private PatternMode currentMode = PatternMode.OFF;

  /** Creates a new WhiteLED subsystem. */
  public AddressableLEDs() {
    setPatternMode(PatternMode.OFF); //Can be changed, should set the pattern on startup
    
    m_led = new AddressableLED(kLedPort);

    // Create a buffer for the LED data.
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data and start the LED output.
    m_led.setData(m_ledBuffer);
    m_led.start();

   

    // Set the default command to turn the strip off, otherwise the last colors written by
    // the last command to run will continue to be displayed.
    // Note: Other default patterns could be used instead!
    // setDefaultCommand(runPattern(LEDPattern.solid(Color.kBlack)).withName("Off")); //Seems this is not needed
  }

  public void setPatternMode(PatternMode mode) {
    currentMode = mode;
  }  

  @Override
  public void periodic() { //Loop for running light patterns. Warning: RUNS WHILE DISABLED

    double maxSpeed = 0.5;
    double minSpeed = 0.2;

    // run current pattern
    switch (currentMode) {
        case FLASHBANG:
            if (Timer.getFPGATimestamp() - lastFlashTime > FLASH_INTERVAL) {
                lastFlashTime = Timer.getFPGATimestamp();
                flashState = !flashState;

                for (int i = 0; i < m_ledBuffer.getLength(); i++) {
                    if (flashState) {
                        m_ledBuffer.setRGB(i, 255, 255, 255); // bright flash
                    } else {
                        m_ledBuffer.setRGB(i, 0, 0, 0); // off
                    }
                }
            }
            break;

        case OFF:
            for (int i = 0; i < m_ledBuffer.getLength(); i++) m_ledBuffer.setRGB(i, 0,0,0);
            break;
    }

    m_led.setData(m_ledBuffer);
  }
}
