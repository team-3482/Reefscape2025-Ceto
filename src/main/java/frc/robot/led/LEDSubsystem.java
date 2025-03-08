package frc.robot.led;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.ShuffleboardTabNames;
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

    private AddressableLED LEDStrip = new AddressableLED(LEDConstants.PWM_HEADER);
    private AddressableLEDBuffer LEDStripBuffer = new AddressableLEDBuffer(LEDConstants.LED_LENGTH);

    private StatusColors currentColor = StatusColors.OFF;
    private StatusColors blinkColor = StatusColors.OFF;

    private boolean shouldBlink = false;
    private Timer blinkTimer = new Timer();

    private Timer stickyTimer = new Timer();

    private SimpleWidget shuffleboard_widget1 = Shuffleboard.getTab(ShuffleboardTabNames.DEFAULT)
        .add("LED 1", false);
    private SimpleWidget shuffleboard_widget2 = Shuffleboard.getTab(ShuffleboardTabNames.DEFAULT)
        .add("LED 2", false);
    @SuppressWarnings("unused")
    private GenericEntry shuffleboard_entry1 = shuffleboard_widget1
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("colorWhenFalse", "black", "colorWhenTrue", "black"))
        .withSize(1, 8)
        .withPosition(0, 0)
        .getEntry();
    @SuppressWarnings("unused")
    private GenericEntry shuffleboard_entry2 = shuffleboard_widget2
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("colorWhenFalse", "black", "colorWhenTrue", "black"))
        .withSize(1, 8)
        .withPosition(18, 0)
        .getEntry();

    /** Creates a new LEDSubsystem. */
    private LEDSubsystem() {
        super("LEDSubsystem");

        this.LEDStrip.setLength(this.LEDStripBuffer.getLength());
        this.LEDStrip.setData(this.LEDStripBuffer);
        this.LEDStrip.start();

        this.blinkTimer.start();
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        if (this.shouldBlink && this.blinkTimer.hasElapsed(LEDConstants.BLINK_COOLDOWN)) {
            if (this.currentColor.equals(StatusColors.OFF)) {
                this.setColor(this.blinkColor, true);
            }
            else {
                this.setColor(StatusColors.OFF, true);
            }
            
            this.blinkTimer.reset();
        }

        // Blink color will always be current color, unless blinking, when it will always be the blink color
        if (this.blinkColor.stickyTime >= 0 && this.stickyTimer.hasElapsed(this.blinkColor.stickyTime)) {
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

        if (!this.shouldBlink || (this.shouldBlink && !newColor.equals(StatusColors.OFF))) {
            this.blinkColor = this.currentColor = newColor; // Don't save black on blinks
        }
        else {
            this.currentColor = newColor;
        }

        LEDPattern pattern = LEDPattern.solid(newColor.color).atBrightness(newColor.brightness);
        pattern.applyTo(this.LEDStripBuffer);
        this.LEDStrip.setData(this.LEDStripBuffer);

        updateShuffleboardsAndLogs(newColor);
    }

    /**
     * Updates Shuffleboard and AdvantageScope logs.
     * @param newColor - The new color to use.
     */
    private void updateShuffleboardsAndLogs(StatusColors newColor) {
        String hexString = newColor.color.toHexString();
        Map<String, Object> properties = Map.of("colorWhenFalse",
            // Have to do this because LEDs don't display orange properly so it is in reality
            // very dark red that our eyes perceive as orange, but in driver station it shows
            // in red, which could confuse the driver. Thus, it has to be modified to orange.
            newColor.equals(StatusColors.RSL) ? Color.kDarkOrange.toHexString() : hexString);
        
        this.shuffleboard_widget1.withProperties(properties);
        this.shuffleboard_widget2.withProperties(properties);

        Logger.recordOutput("LED/Status", newColor);
        Logger.recordOutput("LED/Color", hexString);
    }

    /**
     * Forcefully sets the current color of led strip
     * @param newColor - The color to set.
     */
    public void setColor(StatusColors newColor) {
        // if (DriverStation.isEnabled() || newColor.equals(StatusColors.RSL) || newColor.equals(StatusColors.OFF)) {
            setColor(newColor, false);
        // }
    }

    /**
     * Blinks the color every LEDConsants.BLINK_COOLDOWN
     * @param newColor - The color to blink.
     */
    public void blinkColor(StatusColors newColor) {
        // if (DriverStation.isEnabled() || newColor.equals(StatusColors.RSL)) {
            setColor(newColor, true);
        // }
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
