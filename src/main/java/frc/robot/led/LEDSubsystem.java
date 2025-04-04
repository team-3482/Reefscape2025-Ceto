package frc.robot.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.PhysicalConstants.LEDConstants;

import org.littletonrobotics.junction.Logger;

public class LEDSubsystem extends SubsystemBase {
    // Use Bill Pugh Singleton Pattern for efficient lazy initialization (thread-safe !)
    private static class LEDSubsystemHolder {
        private static final LEDSubsystem INSTANCE = new LEDSubsystem();
    }
    
    public static LEDSubsystem getInstance() {
        return LEDSubsystemHolder.INSTANCE;
    }

    private final AddressableLED LEDStrip = new AddressableLED(LEDConstants.PWM_HEADER);
    private final AddressableLEDBuffer LEDStripBuffer = new AddressableLEDBuffer(LEDConstants.LED_LENGTH);

    private StatusColors currentColor = StatusColors.OFF;
    private StatusColors blinkColor = StatusColors.OFF;

    private boolean shouldBlink = false;
    private final Timer blinkTimer = new Timer();

    private final Timer stickyTimer = new Timer();

    /** Creates a new LEDSubsystem. */
    private LEDSubsystem() {
        super("LEDSubsystem");

        this.LEDStrip.setLength(this.LEDStripBuffer.getLength());
        this.LEDStrip.setData(this.LEDStripBuffer);
        this.LEDStrip.start();

        this.blinkTimer.start();

        SmartDashboard.putString("LED", StatusColors.OFF.color.toHexString());

    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        if (this.shouldBlink && this.blinkTimer.hasElapsed(LEDConstants.BLINK_COOLDOWN)) {
            if (this.currentColor == StatusColors.OFF) {
                this.setColor(this.blinkColor, true);
            }
            else {
                this.setColor(StatusColors.OFF, true);
            }
            
            this.blinkTimer.reset();
        }

        double stickyTime = this.blinkColor.stickyTime;
        // Blink color will always be current color, unless blinking, when it will always be the blink color
        if (stickyTime >= 0 && this.stickyTimer.hasElapsed(stickyTime)) {
            setColor(StatusColors.OFF);
        }
    }

    /**
     * Forcefully sets the current color of led strip.
     * @param newColor - The color to set. Will only be set if the priority is higher than the current one.
     * @param forBlink - Whether the color should be blinked.
     * @see {@link LEDSubsystem#periodic()}
     */
    private void setColor(StatusColors newColor, boolean forBlink) {
        if (newColor.priority != -1 && newColor.priority < this.currentColor.priority) {
            return;
        }

        if (newColor.stickyTime >= 0) {
            this.stickyTimer.restart();
            if (newColor.equals(this.currentColor)) {
                return;
            }
        }
        
        this.shouldBlink = forBlink;

        if (this.shouldBlink) {
            this.blinkTimer.reset();
        }

        LEDPattern pattern = LEDPattern.solid(newColor.color).atBrightness(newColor.brightness);
        pattern.applyTo(this.LEDStripBuffer);
        this.LEDStrip.setData(this.LEDStripBuffer);

        // Don't save black as a blink color on blinks
        if (this.shouldBlink && newColor == StatusColors.OFF) {
            this.currentColor = newColor;
        }
        else {
            this.blinkColor = this.currentColor = newColor;
        }

        updateDashboardAndLogs(newColor);
    }

    /**
     * Updates Dashboard and AdvantageScope logs.
     * @param newColor - The new color to use.
     */
    private void updateDashboardAndLogs(StatusColors newColor) {
        String hexString = newColor.color.toHexString();

        Logger.recordOutput("LED/Status", newColor);
        Logger.recordOutput("LED/Color", hexString);

        SmartDashboard.putString(
            "LED",
            newColor.equals(StatusColors.RSL)
                ? Color.kDarkOrange.toHexString()
                : hexString
        );
    }

    /**
     * Forcefully sets the current color of led strip
     * @param newColor - The color to set.
     */
    public void setColor(StatusColors newColor) {
        setColor(newColor, false);
    }

    /**
     * Blinks the color every LEDConsants.BLINK_COOLDOWN
     * @param newColor - The color to blink.
     */
    public void blinkColor(StatusColors newColor) {
        setColor(newColor, true);
    }

    /**
     * Gets the currently blinking color.
     * @return The color.
     */
    public StatusColors getBlinkColor() {
        return this.blinkColor;
    }

    /** 
     * Gets color of the current strip.
     * @return The color of the strip.
     * @apiNote This uses the FIRST index of the led strip (first node),
     * but this should not be a problem because we always set the whole strip to a solid color.
     * @apiNote Returns null if the color doesn't exist in StatusColors.
     */
    public StatusColors getActualLEDColor() {
        return StatusColors.getColor(this.LEDStripBuffer.getLED(0));
    }
}
