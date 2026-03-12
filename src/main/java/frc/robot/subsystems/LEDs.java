package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import java.security.Identity;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
    AddressableLED m_led;
    AddressableLEDBuffer m_ledBuffer;

    // all hues at maximum saturation and half brightness
    private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);
    // Our LED strip has a density of 120 LEDs per meter
    private static final Distance kLedSpacing = Meters.of(1 / 60.0);

    private final int iterateColor = 12;
    int iterF = 0;
    int iterC = 0;
    double p = 0.0;

    int timer = 51;

    boolean idle = true;
    //fade green to gold
    boolean inv = false;
    //fade colors backwards

    boolean climb = false;

    boolean intake = false;
    boolean outake = false;

    // boolean goldGroup = true;
    // boolean greenGroup = false;

    //LEDS colour states:
    //Disabled: Fade green-yellow-green (repeat)
    //Enabled Hood Down: Solid Green
    //Enabled Hood Down / Intaking: Pulse Green
    //Enabled Hood Up && Flywheel not at speed: Solid White
    //Enabled Hood Up: Solid Yellow
    //Enabled Hood Up / Intaking : Pulse Half Green (Rest unchanged)
    //Enabled Hood Up / Outtaking: Pulse Yellow
    //Climb: Yellow load bar slowly filling up, turn solid green when 100%.

    public LEDs() {
        // PWM port 0
        // Must be a PWM header, not MXP or DIO
        m_led = new AddressableLED(0);

        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data
        m_ledBuffer = new AddressableLEDBuffer(183);
        m_led.setLength(m_ledBuffer.getLength());

        // m_rainbow.applyTo(m_ledBuffer);
    }

    public void funnyPattern() {
        int length = m_ledBuffer.getLength();

        if (timer > 99) {
            timer = 0;
        } else if((intake || outake) && !idle) {
            timer++;
        }

        if(!inv && idle)
            iterF++; 
        else if(idle)
            iterF--;

        for (int i = 0; i < length; i++) {

            if(idle)
                fade(i);
            else if(climb) {
                int j = 0;
                for(j = iterC; j < Math.round(length * p) + iterC; j++) {
                    m_ledBuffer.setLED(j % length, new Color(255, 165, 0)); 
                }
                for(; j < length + iterC; j++) {
                    m_ledBuffer.setLED(j % length, Color.kBlack);
                }
                //get length
                //get percentage done
                //change the leds up to the length multiplied by the percentage (0 <= p <= 1)
                //start at o which iterates every rate
                //changes led at i % l to wrap around
                //sets o to 0 if o == l
            }
            else if(timer < 50) {
                if(intake && !outake)
                    m_ledBuffer.setLED(i, new Color(0, 128, 0));
                else if(outake && !intake)
                    m_ledBuffer.setLED(i, new Color(255, 165, 0));
                else {
                    if((i / (length / 4)) % 2 == 0)
                        m_ledBuffer.setLED(i, new Color(0, 128, 0));
                    else
                        m_ledBuffer.setLED(i, new Color(255, 165, 0)); // Aldershot Gold
                }
            } else {
                if(!intake && !outake)
                    if((i / (length / 4)) % 2 == 0)
                        m_ledBuffer.setLED(i, new Color(0, 128, 0));
                    else
                        m_ledBuffer.setLED(i, new Color(255, 165, 0)); // Aldershot Gold
                else
                    m_ledBuffer.setLED(i - 1, Color.kBlack);
            }
        }

        if(iterF > iterateColor)
            inv = true;
        else if(iterF <= 0)
            inv = false;
    }

    @Override
    public void periodic() {
        funnyPattern();
        m_led.setData(m_ledBuffer);
        m_led.start();
    }


    private void fade(int i) {
        m_ledBuffer.setLED(i, new Color(0 + 255 * (iterF / iterateColor), 128 + 37 * (iterF / iterateColor), 0));
    }
}
