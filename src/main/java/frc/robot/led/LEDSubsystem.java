package frc.robot.led;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.ShuffleboardTabNames;
import frc.robot.constants.Constants.StatusColors;
import frc.robot.constants.PhysicalConstants.LEDConstants;
import org.littletonrobotics.junction.Logger;

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

    /** Used to run timer processing on a separate thread. */
    private final Notifier notifier;

    private volatile AddressableLED LEDStrip = new AddressableLED(LEDConstants.PWM_HEADER);
    private volatile AddressableLEDBuffer LEDStripBuffer = new AddressableLEDBuffer(LEDConstants.LED_LENGTH);

    private volatile StatusColors blinkColor = StatusColors.OFF;
    private volatile boolean shouldBlink = false;
    private volatile Timer blinkTimer = new Timer();
    private volatile Timer stickyTimer = new Timer();
    private volatile double stickyTime = -1;

    private volatile SimpleWidget shuffleboard_widget1 = Shuffleboard.getTab(ShuffleboardTabNames.DEFAULT)
        .add("LED 1", false);
    private volatile SimpleWidget shuffleboard_widget2 = Shuffleboard.getTab(ShuffleboardTabNames.DEFAULT)
        .add("LED 2", false);
    private volatile GenericEntry shuffleboard_entry1 = shuffleboard_widget1
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("colorWhenFalse", StatusColors.OFF.color.toHexString()))
        .withSize(1, 8)
        .withPosition(0, 0)
        .getEntry();
    private volatile GenericEntry shuffleboard_entry2 = shuffleboard_widget2
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("colorWhenFalse", StatusColors.OFF.color.toHexString()))
        .withSize(1, 8)
        .withPosition(18, 0)
        .getEntry();

    /** Creates a new LEDSubsystem. */
    private LEDSubsystem() {
        super("LEDSubsystem");

        this.LEDStrip.setLength(this.LEDStripBuffer.getLength());
        this.LEDStrip.setData(this.LEDStripBuffer);
        this.LEDStrip.start();

        this.notifier = new Notifier(this::notifierLoop);
        this.notifier.setName("LED Notifier");
        this.notifier.startPeriodic(0.05);

        this.blinkTimer.start();
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        // Uses a Notifier for separate-thread timer processing
        // These methods are here because they are NOT thread-safe
        Logger.recordOutput("LED/Status", this.getLEDColor());
        Logger.recordOutput("LED/Color", this.getLEDColor().color.toHexString());
    }

    /**
     * This method is used in conjunction with a Notifier to run timer processing on a separate thread.
     */
    private synchronized void notifierLoop() {
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
    public synchronized StatusColors getLEDColor() {
        return StatusColors.getColor(this.LEDStripBuffer.getLED(0));
    }

    /**
     * Forcefully sets the current color of led strip.
     * @param newColor - The color to set. Will only be set if the priority is higher than the current one.
     * @param forBlink - Whether the color should be blinked.
     * @see {@link LEDSubsystem#periodic()}
     */
    private synchronized void setColor(StatusColors newColor, boolean forBlink) {
        if (newColor.priority != -1 && 
            newColor.priority < (this.shouldBlink ? this.blinkColor.priority : getLEDColor().priority)
        ) return;

        this.shouldBlink = forBlink;

        LEDPattern pattern = LEDPattern.solid(newColor.color);
        pattern.applyTo(this.LEDStripBuffer);
        this.LEDStrip.setData(this.LEDStripBuffer);

        if (newColor.equals(StatusColors.OFF)) {
            this.shuffleboard_entry1.setBoolean(false);
            this.shuffleboard_entry2.setBoolean(false);
        }
        else {
            Map<String, Object> properties = Map.of("colorWhenTrue", newColor.color.toHexString());
            this.shuffleboard_widget1.withProperties(properties);
            this.shuffleboard_widget2.withProperties(properties);
            this.shuffleboard_entry1.setBoolean(true);
            this.shuffleboard_entry2.setBoolean(true);
        }

        this.stickyTime = newColor.stickyTime;
        this.stickyTimer.restart();
    }

    /**
     * Forcefully sets the current color of led strip
     * @param newColor - The color to set.
     */
    public synchronized void setColor(StatusColors newColor) {
        setColor(newColor, false);
    }

    /**
     * Blinks the color every LEDConsants.BLINK_COOLDOWN
     * @param newColor - The color to blink.
     */
    public synchronized void blinkColor(StatusColors newColor) {
        this.blinkColor = newColor;
        setColor(newColor, true);
    }
}
