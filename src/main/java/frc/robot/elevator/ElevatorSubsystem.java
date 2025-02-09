package frc.robot.elevator;

import java.util.Map;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.ShuffleboardTabNames;
import frc.robot.constants.PhysicalConstants.ElevatorConstants;
import frc.robot.constants.PhysicalConstants.RobotConstants;
import frc.robot.constants.PhysicalConstants.ElevatorConstants.ElevatorSlot0Gains;

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

    private DigitalInput lowerLimitSwitch = new DigitalInput(ElevatorConstants.BOTTOM_LIMIT_ID);
    private DigitalInput upperLimitSwitch = new DigitalInput(ElevatorConstants.UPPER_LIMIT_ID);

    private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

    /** Shuffleboard stuff */
    private final ShuffleboardLayout shuffleboardLayout = Shuffleboard.getTab(ShuffleboardTabNames.DEFAULT)
        .getLayout("ElevatorSubsystem", BuiltInLayouts.kGrid)
        .withProperties(Map.of("Number of columns", 1, "Number of rows", 1, "Label position", "TOP"))
        .withSize(4, 2);
    private GenericEntry shuffleboardPositionNumberBar = shuffleboardLayout
        .add("Elevator Position Number Bar", 0)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("Min", 0, "Max", 1, "Show Value", true))
        .withSize(4, 2)
        .withPosition(0, 0)
        .getEntry();
    private GenericEntry shuffleboardTopSensorBoolean = shuffleboardLayout
        .add("Top Sensor", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(0, 1)
        .withSize(2, 2)
        .getEntry();
    private GenericEntry shuffleboardBottomSensorBoolean = shuffleboardLayout
        .add("Bottom Sensor", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(0, 2)
        .withSize(2, 2)
        .getEntry();
    

    /** Creates a new ExampleSubsystem. */
    private ElevatorSubsystem() {
        super("ElevatorSubsystem");

        this.configureMotors();

        this.rightMotor.getPosition().setUpdateFrequency(50);
        this.rightMotor.getVelocity().setUpdateFrequency(50);

        this.leftMotor.setControl(new Follower(ElevatorConstants.RIGHT_MOTOR_ID, true));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        String controlName = this.rightMotor.getAppliedControl().getControlInfo().get("Name");

        System.out.printf("Position : %f ; Velocity : %f%n", getPosition(), getRotorVelocity());

        this.shuffleboardPositionNumberBar.setDouble(getPosition());
        this.shuffleboardTopSensorBoolean.setBoolean(atUpperLimit());
        this.shuffleboardBottomSensorBoolean.setBoolean(atLowerLimit());

        if (controlName.equals("VoltageOut")) {
            this.rightMotor.setControl(((VoltageOut) this.rightMotor.getAppliedControl())
                .withLimitForwardMotion(this.upperLimitSwitch.get())
                .withLimitReverseMotion(this.lowerLimitSwitch.get())
            );
        }
        else if (controlName.equals("MotionMagicVoltage")) {
            this.rightMotor.setControl(((MotionMagicVoltage) this.rightMotor.getAppliedControl())
                .withLimitForwardMotion(this.upperLimitSwitch.get())
                .withLimitReverseMotion(this.lowerLimitSwitch.get())
            );
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

        // Motor-specific configurations.
        motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive; // Right motor not inverted.
        this.rightMotor.getConfigurator().apply(configuration);

        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive; // Left motor inverted.
        this.leftMotor.getConfigurator().apply(configuration);
    }

    /**
     * Sets the position of the elevator in meters.
     * @param position - The position in meters.
     */
    public void setPosition(double position) {
        position = this.metersToRotation(position);
        this.rightMotor.setPosition(position);
        // this.leftMotor.setPosition(position);
    }

    /**
     * Gets the position of the elevator in meters.
     * @return The position in meters.
     */
    public double getPosition() {
        return this.rotationsToMeters(this.rightMotor.getPosition().getValueAsDouble());
    }


    public double getRotorVelocity() {
        return this.rightMotor.getVelocity().getValueAsDouble();
    }

    /**
     * Moves the elevator to a position in meters using Motion Magic.
     * @param position - The position in meters.
     * @param clamp - Whether to clamp the position to the soft stops.
     */
    public void motionMagicPosition(double position, boolean clamp) {
        if (clamp) {
            position = MathUtil.clamp(position, ElevatorConstants.LOWER_STOP, ElevatorConstants.UPPER_STOP);
        }

        MotionMagicVoltage control = motionMagicVoltage
            .withSlot(0)
            .withPosition(this.metersToRotation(position));   

        this.rightMotor.setControl(control
            .withLimitReverseMotion(this.lowerLimitSwitch.get())
            .withLimitForwardMotion(this.upperLimitSwitch.get())
        );
    }

    /**
     * Sends a voltage to the motor
     * @param voltage - Voltage in between 12 and -12
     */
    public void setVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -12, 12);

        rightMotor.setControl(new VoltageOut(voltage)
            .withLimitForwardMotion(this.upperLimitSwitch.get())
            .withLimitReverseMotion(this.lowerLimitSwitch.get())
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
     * Checks if the current elevator position is at the top sensor
     * @return Whether the current position is at the top  sensor
     */
    public boolean atUpperLimit(){
        return this.upperLimitSwitch.get();
    }

    /**
     * Checks if the current elevator position is at the bottom sensor
     * @return Whether the current position is at the bottom sensor
     */
    public boolean atLowerLimit(){
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
