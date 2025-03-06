package frc.robot.elevator;

import java.util.Map;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.ScoringConstants;
import frc.robot.constants.Constants.ShuffleboardTabNames;
import frc.robot.constants.PhysicalConstants.ElevatorConstants;
import frc.robot.constants.PhysicalConstants.RobotConstants;
import frc.robot.constants.PhysicalConstants.ElevatorConstants.ElevatorSlot0Gains;
import org.littletonrobotics.junction.Logger;

/** A subsystem that moves the elevator up and down using MotionMagic. */
public class ElevatorSubsystem extends SubsystemBase {
    // Thread-safe singleton design pattern.
    private static volatile ElevatorSubsystem instance;
    private static Object mutex = new Object();

    public static ElevatorSubsystem getInstance() {
        ElevatorSubsystem result = instance;
       
        if (result == null) {
            synchronized (mutex) {
                result = instance;
                if (result == null) {
                    instance = result = new ElevatorSubsystem();
                }
            }
        }
        return instance;
    }

    private TalonFX leftMotor = new TalonFX(ElevatorConstants.LEFT_MOTOR_ID, RobotConstants.CTRE_CAN_BUS);
    private TalonFX rightMotor = new TalonFX(ElevatorConstants.RIGHT_MOTOR_ID, RobotConstants.CTRE_CAN_BUS);

    private DigitalInput lowerLimitSwitch = new DigitalInput(ElevatorConstants.BOTTOM_LASER_ID);
    private DigitalInput stageTwoUpperLimitSwitch = new DigitalInput(ElevatorConstants.STAGE_TWO_UPPER_LASER_ID);
    private DigitalInput stageThreeUpperLimitSwitch = new DigitalInput(ElevatorConstants.STAGE_THREE_UPPER_LASER_ID);

    private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);
    private final Follower FOLLOW_RIGHT = new Follower(ElevatorConstants.RIGHT_MOTOR_ID, true);
    private final Follower FOLLOW_LEFT = new Follower(ElevatorConstants.LEFT_MOTOR_ID, true);

    /** Shuffleboard stuff */
    private final ShuffleboardLayout shuffleboardLayout = Shuffleboard.getTab(ShuffleboardTabNames.DEFAULT)
        .getLayout("ElevatorSubsystem", BuiltInLayouts.kGrid)
        .withProperties(Map.of("Number of columns", 1, "Number of rows", 6, "Label position", "TOP"))
        .withSize(5, 8)
        .withPosition(13, 0);
    private GenericEntry shuffleboardPositionNumberBar = shuffleboardLayout
        .add("Elevator Position (meters)", 0)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("Min", ScoringConstants.BOTTOM_HEIGHT, "Max", ScoringConstants.MAX_HEIGHT, "Num tick marks", 0))
        .withSize(5, 2)
        .withPosition(0, 0)
        .getEntry();
    private GenericEntry shuffleboardToggleInput = shuffleboardLayout
        .add("Toggle Shuffleboard Input Slider", false)
        .withWidget(BuiltInWidgets.kToggleButton)
        .withSize(5, 1)
        .withPosition(0, 1)
        .getEntry();
    private GenericEntry shuffleboardSliderInput = shuffleboardLayout
        .add("Set Elevator Position (meters)", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("Min", ScoringConstants.BOTTOM_HEIGHT, "Max", ScoringConstants.MAX_HEIGHT, "Block increment", 0.01))
        .withSize(5, 2)
        .withPosition(0, 2)
        .getEntry();
    private GenericEntry shuffleboardStageThreeTopSensor = shuffleboardLayout
        .add("Stage Three Top Sensor", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withSize(5, 1)
        .withPosition(0, 3)
        .getEntry();
    private GenericEntry shuffleboardStageTwoTopSensor = shuffleboardLayout
        .add("Stage Two Top Sensor", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withSize(5, 1)
        .withPosition(0, 4)
        .getEntry();
    private GenericEntry shuffleboardBottomSensorBoolean = shuffleboardLayout
        .add("Bottom Sensor", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withSize(5, 1)
        .withPosition(0, 5)
        .getEntry();

    /** Creates a new ElevatorSubsystem. */
    private ElevatorSubsystem() {
        super("ElevatorSubsystem");

        this.configureMotors();

        this.rightMotor.getPosition().setUpdateFrequency(50);
        this.rightMotor.getVelocity().setUpdateFrequency(50);

        this.leftMotor.setControl(this.FOLLOW_RIGHT);

        if (atLowerLimit()) {
            // If starting at the bottom (beginning of a match, etc.) reset the position
            // We do not do this every time because sometimes we deploy code while the elevator
            // is being worked on by mech, etc. and in that case the position is correct
            // because it is using the last reset
            setPosition(ScoringConstants.BOTTOM_HEIGHT);
        }
    }

    @Override
    // This method will be called once per scheduler run
    public void periodic() {
        this.shuffleboardPositionNumberBar.setDouble(getPosition());
        this.shuffleboardStageThreeTopSensor.setBoolean(atUpperLimit_StageThree());
        this.shuffleboardStageTwoTopSensor.setBoolean(atUpperLimit_StageTwo());
        this.shuffleboardBottomSensorBoolean.setBoolean(atLowerLimit());

        Logger.recordOutput("Elevator/Position", this.getPosition());
        Logger.recordOutput("Elevator/RotorVelocity", this.getRotorVelocity());

        boolean inputToggled = this.shuffleboardToggleInput.getBoolean(false);

        if (!inputToggled) {
            this.shuffleboardSliderInput.setDouble(getPosition());
        }
        
        if (DriverStation.isEnabled()) {
            ControlRequest appliedControl = this.rightMotor.getAppliedControl();
            String controlName = appliedControl.getControlInfo().get("Name");
            boolean leftMotor = false;

            if (controlName.equals("Follower")) {
                appliedControl = this.leftMotor.getAppliedControl();
                controlName = appliedControl.getControlInfo().get("Name");
                leftMotor = true;
            }

            if (controlName.equals("VoltageOut")) {
                if (leftMotor) {
                    this.leftMotor.setControl(((VoltageOut) appliedControl)
                        .withLimitForwardMotion(atUpperLimit())
                        .withLimitReverseMotion(atLowerLimit())
                    );
                    this.rightMotor.setControl(this.FOLLOW_LEFT);
                }
                else {
                    this.rightMotor.setControl(((VoltageOut) appliedControl)
                        .withLimitForwardMotion(atUpperLimit())
                        .withLimitReverseMotion(atLowerLimit())
                    );
                    this.leftMotor.setControl(this.FOLLOW_RIGHT);
                }
            }
            else if (controlName.equals("MotionMagicVoltage")) {
                if (leftMotor) {
                    this.leftMotor.setControl(((MotionMagicVoltage) appliedControl)
                        .withLimitForwardMotion(atUpperLimit())
                        .withLimitReverseMotion(atLowerLimit())
                    );
                    this.rightMotor.setControl(this.FOLLOW_LEFT);
                }
                else {
                    this.rightMotor.setControl(((MotionMagicVoltage) appliedControl)
                        .withLimitForwardMotion(atUpperLimit())
                        .withLimitReverseMotion(atLowerLimit())
                    );
                    this.leftMotor.setControl(this.FOLLOW_RIGHT);
                }
            }

            Command currentCommand = getCurrentCommand();

            if (atUpperLimit()) {
                if (currentCommand != null) {
                    CommandScheduler.getInstance().cancel(currentCommand);
                }
                motionMagicPosition(getPosition() - 0.01, false, true);
            }
            else if (currentCommand != null) {
                this.shuffleboardToggleInput.setBoolean(false);
            }
            else if (inputToggled && currentCommand == null) {
                motionMagicPosition(this.shuffleboardSliderInput.getDouble(getPosition()), true, false);
            }
        }
        else {
            this.shuffleboardToggleInput.setBoolean(false);
            setVoltage(0);
        }
    }

    /**
     * A helper method that configures MotionMagic on both motors.
     */
    private void configureMotors() {
        TalonFXConfiguration configuration = new TalonFXConfiguration();

        FeedbackConfigs feedbackConfigs = configuration.Feedback;
        feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        // Sets the gear ration from the rotor to the mechanism.
        // This gear ratio needs to be exact.
        feedbackConfigs.SensorToMechanismRatio = ElevatorConstants.ROTOR_TO_MECHANISM_RATIO; 

        MotorOutputConfigs motorOutputConfigs = configuration.MotorOutput;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

        // Set Motion Magic gains in slot 0.
        Slot0Configs slot0Configs = configuration.Slot0;
        slot0Configs.GravityType = GravityTypeValue.Elevator_Static;
        slot0Configs.kG = ElevatorSlot0Gains.kG;
        slot0Configs.kS = ElevatorSlot0Gains.kS;
        slot0Configs.kV = ElevatorSlot0Gains.kV;
        slot0Configs.kA = ElevatorSlot0Gains.kA;
        slot0Configs.kP = ElevatorSlot0Gains.kP;
        slot0Configs.kI = ElevatorSlot0Gains.kI;
        slot0Configs.kD = ElevatorSlot0Gains.kD;

        // Set acceleration and cruise velocity.
        MotionMagicConfigs motionMagicConfigs = configuration.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ElevatorConstants.CRUISE_SPEED;
        motionMagicConfigs.MotionMagicAcceleration = ElevatorConstants.ACCELERATION;
        // motionMagicConfigs.MotionMagicJerk = PivotConstants.MOTION_MAGIC_JERK;

        CurrentLimitsConfigs currentLimitsConfigs = configuration.CurrentLimits;
        currentLimitsConfigs.StatorCurrentLimitEnable = true;
        currentLimitsConfigs.StatorCurrentLimit = 50;
        currentLimitsConfigs.SupplyCurrentLimitEnable = true;
        currentLimitsConfigs.SupplyCurrentLimit = 30;
        currentLimitsConfigs.SupplyCurrentLowerTime = 1;
        currentLimitsConfigs.SupplyCurrentLowerLimit = 25;

        // Motor-specific configurations.
        motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive; // Right motor not inverted.
        this.rightMotor.getConfigurator().apply(configuration);

        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive; // Left motor inverted.
        motionMagicConfigs.MotionMagicCruiseVelocity = ElevatorConstants.SLOW_CRUISE_SPEED;
        motionMagicConfigs.MotionMagicAcceleration = ElevatorConstants.SLOW_ACCELERATION;
        this.leftMotor.getConfigurator().apply(configuration);
    }

    /**
     * Sets the position of the elevator in meters.
     * @param position - The position in meters.
     */
    public void setPosition(double position) {
        this.rightMotor.setPosition(metersToRotation(position));
        this.leftMotor.setPosition(metersToRotation(position));
    }

    /**
     * Gets the position of the elevator in meters.
     * @return The position in meters.
     */
    public double getPosition() {
        return this.rotationsToMeters(this.rightMotor.getPosition().getValueAsDouble());
    }

    /**
     * Returns the velocity of the motor with no unit conversions
     * @return The velocity of the motors
     */
    public double getRotorVelocity() {
        return this.rightMotor.getVelocity().getValueAsDouble();
    }

    /**
     * Moves the elevator to a position in meters using Motion Magic.
     * @param position - The position in meters.
     * @param clamp - Whether to clamp the position to the soft stops.
     * @param slow - Whether or not to limit the elevator's max speed and acceleration.
     */
    public void motionMagicPosition(double position, boolean clamp, boolean slow) {
        if (clamp) {
            position = MathUtil.clamp(position, ScoringConstants.BOTTOM_HEIGHT, ScoringConstants.MAX_HEIGHT);
        }

        MotionMagicVoltage control = motionMagicVoltage
            .withSlot(0)
            .withPosition(this.metersToRotation(position));

        if (slow) {
            this.leftMotor.setControl(control
                .withLimitForwardMotion(atUpperLimit())
                .withLimitReverseMotion(atLowerLimit())
            );
            this.rightMotor.setControl(this.FOLLOW_LEFT);
        }
        else {
            this.rightMotor.setControl(control
                .withLimitForwardMotion(atUpperLimit())
                .withLimitReverseMotion(atLowerLimit())
            );
            this.leftMotor.setControl(this.FOLLOW_RIGHT);
        }
    }

    /**
     * Sends a voltage to the motor
     * @param voltage - Voltage in between 12 and -12
     */
    public void setVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -12, 12);

        rightMotor.setControl(new VoltageOut(voltage)
            .withLimitForwardMotion(atUpperLimit())
            .withLimitReverseMotion(atLowerLimit())
        );
    }

    /**
     * Checks if the current elevator position is within a tolerance of a position.
     * @param position - The position to check against.
     * @return Whether the current position is within the tolerance.
     */
    public boolean withinTolerance(double position) {
        return Math.abs(getPosition() - position) <= ElevatorConstants.HEIGHT_TOLERANCE;
    }

    /**
     * Checks if the second stage is at its highest point.
     * @return Whether the second stage is at its highest point.
     */
    private boolean atUpperLimit_StageTwo() {
        return this.stageTwoUpperLimitSwitch.get();
    }

    /**
     * Checks if the third stage is at its highest point.
     * @return Whether the third stage is at its highest point.
     */
    private boolean atUpperLimit_StageThree() {
        return this.stageThreeUpperLimitSwitch.get();
    }

    /**
     * Checks if the current elevator position is at the top sensors
     * @return Whether the current position is at the top  sensor
     */
    public boolean atUpperLimit() {
        return atUpperLimit_StageTwo() && atUpperLimit_StageThree();
    }

    /**
     * Checks if the current elevator position is at the bottom sensor
     * @return Whether the current position is at the bottom sensor
     */
    public boolean atLowerLimit() {
        return this.lowerLimitSwitch.get();
    }

    /**
     * Converts motor rotations to elevator meters.
     * @param rotations - The rotations to convert.
     * @return The meters.
     * @see {@link ElevatorConstants#ROLLER_DIAMETER}
     * @apiNote This method is private and not used by other classes.
     */
    private double rotationsToMeters(double rotations) {
        return Math.PI * ElevatorConstants.ROLLER_DIAMETER * rotations * ElevatorConstants.LINEAR_CONSTANT_MULT;
    }

    /**
     * Converts elevator meters to motor rotations.
     * @param meters - The meters to convert.
     * @return The rotations.
     * @see {@link ElevatorConstants#ROLLER_DIAMETER}
     * @apiNote This method is private and not used by other classes.
     */
    private double metersToRotation(double meters) {
        return meters / ElevatorConstants.ROLLER_DIAMETER / Math.PI / ElevatorConstants.LINEAR_CONSTANT_MULT;
    }
}
