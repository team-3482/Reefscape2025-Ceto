package frc.robot.led;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.io.Console;
import java.util.Vector;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
    // Thread-safe singleton design pattern.
    private static volatile LEDSubsystem instance;
    private static Object mutex = new Object();

    private AddressableLED LEDStrip = new AddressableLED(LEDConstants.PWM_HEADER);
    private AddressableLEDBuffer LEDStripBuffer = new AddressableLEDBuffer(LEDConstants.LED_LENGTH);
    private Color blinkColor = Color.kBlack;
    private boolean shouldBlink = false;
    private Timer blinkTimer = new Timer();

    public static LEDSubsystem getInstance() {
        LEDSubsystem result = instance;
       
        if (result == null) {
            synchronized (mutex) {
                result = instance;
                if (result == null) {
                    instance = result = new LEDSubsystem();
                }
            }
        }
        return instance;
    }


    /** Creates a new ExampleSubsystem. */
    private LEDSubsystem() {
        super("LEDSubsystem");

        this.LEDStrip.setLength(this.LEDStripBuffer.getLength());
        this.LEDStrip.setData(this.LEDStripBuffer);
        this.LEDStrip.start();

        this.blinkTimer.start();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        if (this.shouldBlink) {
            boolean timeElapsed = this.blinkTimer.hasElapsed(LEDConstants.BLINK_COOLDOWN);

            if (timeElapsed) {
                Color color = this.getColor(); // store so we dont unnesscary call it

                if (color.equals(Color.kBlack)) {
                    this.setColor(this.blinkColor, true);
                }
                else {
                    this.setColor(Color.kBlack, true);
                }
                
                this.blinkTimer.reset();
            }
        }
    }

    /** Gets color of the current strip. Note, this uses the FIRST index of the led strip (first note),
     *  yet this should not be a problem because we always set the whole strip to a solid color */
    public Color getColor() {
        return this.LEDStripBuffer.getLED(0);
    }

    /** Forcefully sets the current color of led strip. this is private for the blink functionality */
    private void setColor(Color newColor, boolean forBlink) {
        this.shouldBlink = forBlink;

        LEDPattern color = LEDPattern.solid(newColor);
        color.applyTo(this.LEDStripBuffer);
        this.LEDStrip.setData(this.LEDStripBuffer);
    }

    /** Forcefully sets the current color of led strip */
    public void setColor(Color newColor) {
        this.shouldBlink = false;

        LEDPattern color = LEDPattern.solid(newColor);
        color.applyTo(this.LEDStripBuffer);
        this.LEDStrip.setData(this.LEDStripBuffer);
    }

    /** Blinks the color every LEDConsants.BLINK_COOLDOWN */
    public void blinkColor(Color newColor) {
        this.blinkColor = newColor;
        this.shouldBlink = true;
    }
}
