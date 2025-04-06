package frc.robot.led;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.util.Color;

import java.util.HashMap;
import java.util.Map;

import static edu.wpi.first.units.Units.Percent;

/** Colors used with the LEDSubsystem, these are named for readability and priority. */
public enum StatusColors {
    // Default percent is 50
    OFF(Color.kBlack, null, -1, -1),
    CORAL(Color.kWhite, null, 0, 0.2),
    CAN_ALIGN(Color.kBlue, null, 1, 0.2),
    ERROR(Color.kRed, null, 2, 1),
    OK(Color.kGreen, null, 2, 2.5),
    RSL(new Color(200, 50, 0), Percent.of(40), 5, -1),
    ;
    
    /** The Color. */
    public final Color color;
    /** The priority of this color over other colors (higher = higher priority). */
    public final int priority;
    /** How long this color should stay. -1 is infinitely long. */
    public final double stickyTime;
    /** The brightness to run this color at. */
    public final Dimensionless brightness;

     // Precompute the color map for fast lookup
    private static final Map<Color, StatusColors> COLOR_MAP = new HashMap<>();
    
    static {
        for (StatusColors statusColor : values()) {
            COLOR_MAP.put(statusColor.color, statusColor);
        }
    }

    /**
     * Creates a new StatusColors.
     * @param color - The Color.
     * @param brightness - How bright this color should be. If null, it will be 50%.
     * @param priority - The priority of this color over other colors (higher = higher priority).
     * -1 will always be overriden and will always override.
     * @param stickyTime - How long this color should stay. -1 is infinitely long.
     */
    private StatusColors(Color color, Dimensionless brightness, int priority, double stickyTime) {
        this.color = color;
        this.brightness = brightness == null ? Percent.of(75) : brightness;
        this.priority = priority;
        this.stickyTime = stickyTime;
    }

    /**
     * Get the StatuColors associated with this color.
     * @param color - The color to fetch.
     * @return The StatusColors. If none, returns null.
     */
    public static StatusColors getColor(Color color) {
        return StatusColors.COLOR_MAP.getOrDefault(color, null);
    }
}