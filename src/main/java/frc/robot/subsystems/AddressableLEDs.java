package frc.robot.subsystems;

import java.io.Serial;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

/**
 * A simple subsystem to control an addressable LED strip.
 * The strip needs to be set up in 1 chain, from start to end. Y splits will
 * cause both strips to show the same pattern.
 * Should be plugged into the pwm 0 port on the roboRIO, and the ground and
 * power on the PDH.
 */
public class AddressableLEDs extends SubsystemBase {

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(kLedStripLength);
  private boolean enabled;

  // PWM port for the LED strip. This must be a PWM header on the roboRIO.
  private static final int kLedPort = 0;

  private static final int longLedStrip = 144;
  private static final int shortLedStrip1 = 26;
  private static final int shortLedStrip2 = 26;
  // Number of LEDs on the strip.
  private static final int kLedStripLength = /*longLedStrip + */shortLedStrip1 + shortLedStrip2;

  // AddressableLEDBufferView m_LedSection1 = m_ledBuffer.createView(0, longLedStrip - 1); // Sections of controllable
                                                                                        // LEDs, can be used for
                                                                                        // different patterns/effects -
  // AddressableLEDBufferView m_LedSection2 = m_ledBuffer.createView(longLedStrip, (longLedStrip - 1) + shortLedStrip1); // This
                                                                                                                      // should
                                                                                                                      // probably
                                                                                                                      // be
                                                                                                                      // split
                                                                                                                      // where
                                                                                                                      // the
                                                                                                                      // strips
                                                                                                                      // are
                                                                                                                      // split
                                                                                                                      // and
                                                                                                                      // conected
                                                                                                                      // with
                                                                                                                      // wires.
  // AddressableLEDBufferView m_LedSection3 = m_ledBuffer.createView((longLedStrip - 1) + shortLedStrip1,
      // (longLedStrip - 1) + shortLedStrip1 + shortLedStrip2); // Also sections may need to be reversed to adjust for
                                                             // wiring direction
  // AddressableLEDBufferView m_LedSection4 = m_ledBuffer.createView(164, 182);

  // set up patterns

  public static LEDPattern m_off = LEDPattern.kOff;

  // Variables for patterns

  // flashbang
  private boolean flashState = false;
  private double lastFlashTime = 0;
  private static final double FLASH_INTERVAL = 0.05; // seconds between flashes
  // green/gold blinking pattern
  private double lastChangeTime = 0;
  private static final float colourChangeTime = 1; // time it waits for before changing colour in seconds
  private static final Color ALDgold = new Color(255, 165, 0); // Aldershot Gold
  private static final Color ALDgreen = Color.kDarkGreen; // Aldershot Green
  private static final Color[] Colours = new Color[] { ALDgold, ALDgreen };
  private int currentColour = 0;
  private int ledGroupCounter = 0;

  public enum PatternMode {
    FLASHBANG,
    PATTERN0,
    OFF
  }

  private PatternMode currentMode = PatternMode.OFF;

  /** Creates a new WhiteLED subsystem. */
  public AddressableLEDs(boolean enabled) {
    this.enabled = enabled;

    setPatternMode(PatternMode.OFF); // Can be changed, should set the pattern on startup

    m_led = new AddressableLED(kLedPort);

    // Create a buffer for the LED data.
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data and start the LED output.
    if (enabled) {
      m_led.setData(m_ledBuffer);
      m_led.start();
    }

    // Set the default command to turn the strip off, otherwise the last colors
    // written by
    // the last command to run will continue to be displayed.
    // Note: Other default patterns could be used instead!
    // setDefaultCommand(runPattern(LEDPattern.solid(Color.kBlack)).withName("Off"));
    // //Seems this is not needed
  }

  public void setPatternMode(PatternMode mode) {
    currentMode = mode;
  }

  @Override
  public void periodic() { // Loop for running light patterns. Warning: RUNS WHILE DISABLED
    if (enabled) {

      double maxSpeed = 0.5;
      double minSpeed = 0.2;

      // run current pattern
      switch (currentMode) {
        // case FLASHBANG:
        //   if (Timer.getFPGATimestamp() - lastFlashTime > FLASH_INTERVAL) {
        //     lastFlashTime = Timer.getFPGATimestamp();
        //     flashState = !flashState;

        //     for (int i = 0; i < m_ledBuffer.getLength(); i++) {
        //       if (flashState) {
        //         if (i < m_LedSection1.getLength()) {
        //           m_ledBuffer.setRGB(i, 255, 255, 255); // bright flash
        //         } else if (m_LedSection1.getLength() - 1 < i
        //             && i < m_LedSection1.getLength() + m_LedSection2.getLength()) {
        //           m_ledBuffer.setRGB(i, 0, 255, 0); // bright flash
        //         } else {
        //           m_ledBuffer.setRGB(i, 0, 0, 255); // bright flash
        //         }
        //       } else {
        //         m_ledBuffer.setRGB(i, 0, 0, 0); // off
        //       }
        //     }
        //   }
        //   break;

        case PATTERN0:
          if (Timer.getFPGATimestamp() - lastChangeTime > colourChangeTime) {
            // for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            // m_ledBuffer.setLED(i, Color.kBlack);
            // }

            lastChangeTime = Timer.getFPGATimestamp();
            currentColour = (currentColour + 1) % Colours.length;

            for (int i = 0; i < m_ledBuffer.getLength(); i++) {
              m_ledBuffer.setLED(i, Colours[currentColour]);
              if (ledGroupCounter < 4) {
                ledGroupCounter++;
                // System.out.println("led set to " + Colours[currentColour].toString());
              } else {
                currentColour = (currentColour + 1) % Colours.length;
                ledGroupCounter = 0;
                // System.out.println("colour changed to " + Colours[currentColour].toString());
              }
            }
          }

          break;

        case OFF:
          for (int i = 0; i < m_ledBuffer.getLength(); i++)
            m_ledBuffer.setRGB(i, 0, 0, 0);
          break;
      }

      m_led.setData(m_ledBuffer);
    }
  }
}
