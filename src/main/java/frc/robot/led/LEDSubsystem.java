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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.StatusColors;
import frc.robot.constants.Constants.ShuffleboardTabNames;
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

    private StatusColors blinkColor = StatusColors.OFF;
    private boolean shouldBlink = false;
    private Timer blinkTimer = new Timer();
    private Timer stickyTimer = new Timer();
    private double stickyTime = -1;

    private SimpleWidget shuffleboard_widget = Shuffleboard.getTab(ShuffleboardTabNames.DEFAULT)
        .add("LED", false);
    private GenericEntry shuffleboard_entry = shuffleboard_widget
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("colorWhenFalse", StatusColors.OFF.color.toHexString()))
        .withSize(2, 2)
        .getEntry();

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
            if (this.getLEDColor().equals(StatusColors.OFF)) {
                this.setColor(this.blinkColor, true);
            }
            else {
                this.setColor(StatusColors.OFF, true);
            }
            
            this.blinkTimer.reset();
        }
        else if (this.stickyTime >= 0 && this.stickyTimer.hasElapsed(this.stickyTime)) {
            setColor(StatusColors.OFF);
        }
    }

    /** 
     * Gets color of the current strip.
     * @return The color of the strip.
     * @apiNote This uses the FIRST index of the led strip (first node),
     * but this should not be a problem because we always set the whole strip to a solid color.
     * @apiNote Returns null if the color doesn't exist in StatusColors.
     */
    public StatusColors getLEDColor() {
        return StatusColors.getColor(this.LEDStripBuffer.getLED(0));
    }

    /**
     * Forcefully sets the current color of led strip.
     * @param newColor - The color to set. Will only be set if the priority is higher than the current one.
     * @param forBlink - Whether the color should be blinked.
     * @see {@link LEDSubsystem#periodic()}
     */
    private void setColor(StatusColors newColor, boolean forBlink) {
        if (newColor.priority != -1 && newColor.priority < getLEDColor().priority) return;

        this.shouldBlink = forBlink;

        LEDPattern pattern = LEDPattern.solid(newColor.color);
        pattern.applyTo(this.LEDStripBuffer);
        this.LEDStrip.setData(this.LEDStripBuffer);

        if (newColor.equals(StatusColors.OFF)) {
            this.shuffleboard_entry.setBoolean(false);
        }
        else {
            this.shuffleboard_widget.withProperties(Map.of("colorWhenTrue", newColor.color.toHexString()));
            this.shuffleboard_entry.setBoolean(true);
        }

        this.stickyTime = newColor.stickyTime;
        this.stickyTimer.restart();
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
        this.blinkColor = newColor;
        setColor(newColor, true);
    }
}
