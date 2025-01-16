package frc.robot.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PhysicalConstants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
    // Thread-safe singleton design pattern.
    private static volatile LEDSubsystem instance;
    private static Object mutex = new Object();

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

    private AddressableLED LEDStrip = new AddressableLED(LEDConstants.PWM_HEADER);
    private AddressableLEDBuffer LEDStripBuffer = new AddressableLEDBuffer(LEDConstants.LED_LENGTH);

    private Color blinkColor = Color.kBlack;
    private boolean shouldBlink = false;
    private Timer blinkTimer = new Timer();

    /** Creates a new LEDSubsystem. */
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

        if (this.shouldBlink && this.blinkTimer.hasElapsed(LEDConstants.BLINK_COOLDOWN)) {
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

    /** 
     * Gets color of the current strip.
     * @return The color of the strip.
     * @apiNote This uses the FIRST index of the led strip (first note),
     * but this should not be a problem because we always set the whole strip to a solid color.
     */
    public Color getColor() {
        return this.LEDStripBuffer.getLED(0);
    }

    /**
     * Forcefully sets the current color of led strip.
     * @param newColor - The color to set.
     * @param forBlink - Whether the color should be blinked.
     * @see {@link LEDSubsystem#periodic()}
     */
    private void setColor(Color newColor, boolean forBlink) {
        this.shouldBlink = forBlink;

        LEDPattern color = LEDPattern.solid(newColor);
        color.applyTo(this.LEDStripBuffer);
        this.LEDStrip.setData(this.LEDStripBuffer);
    }

    /**
     * Forcefully sets the current color of led strip
     * @param newColor - The color to set.
     */
    public void setColor(Color newColor) {
        setColor(newColor, false);
    }

    /**
     * Blinks the color every LEDConsants.BLINK_COOLDOWN
     * @param newColor - The color to blink.
     */
    public void blinkColor(Color newColor) {
        setColor(newColor, true);
    }
}
