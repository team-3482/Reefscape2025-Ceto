package frc.robot.elevator;

import java.util.Map;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Notifier;
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
    private DigitalInput bottomLimit = new DigitalInput(ElevatorConstants.BOTTOM_LIMIT_ID);
    private DigitalInput upperLimit = new DigitalInput(ElevatorConstants.UPPER_LIMIT_ID);
    private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

    private final ShuffleboardLayout shuffleboardLayout = Shuffleboard.getTab(ShuffleboardTabNames.DEFAULT)
        .getLayout("PivotSubsystem", BuiltInLayouts.kGrid)
        .withProperties(Map.of("Number of columns", 1, "Number of rows", 1, "Label position", "TOP"))
        .withSize(4, 1)
        .withPosition(0, 0);
    private GenericEntry shuffleboardPositionNumberBar = shuffleboardLayout
        .add("Pivot Position Dial", 0)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("Min", 0, "Max", 1, "Show Value", true))
        .withSize(4, 1)
        .withPosition(0, 0)
        .getEntry();

    /** Creates a new ExampleSubsystem. */
    private ElevatorSubsystem() {
        super("ElevatorSubsystem");

        this.configureMotors();

        this.rightMotor.getPosition().setUpdateFrequency(50);

        this.leftMotor.setControl(new Follower(ElevatorConstants.RIGHT_MOTOR_ID, true));
        final DutyCycleOut m_dutyCycle = new DutyCycleOut(0.0);

        // this.notifier = new Notifier(() -> notifierLoop());
        // this.notifier.setName("Pivot Notifier");
        // // 100 ms cycle, or 5 robot cycles.
        // this.notifier.startPeriodic(0.1);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        double position = getPosition();
        this.shuffleboardPositionNumberBar.setDouble(position);
    }

    // private synchronized void notifierLoop() {
    //     double position = getPosition();
    //     this.shuffleboardPositionNumberBar.setDouble(position);
    // }

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
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive; // Right motor not inverted.
        this.rightMotor.getConfigurator().apply(configuration);

        motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive; // Left motor inverted.
        this.leftMotor.getConfigurator().apply(configuration);
    }

    public void setPosition(double position) {
        position = this.metersToRotation(position);
        this.rightMotor.setPosition(position);
        this.leftMotor.setPosition(position);
    }

    public double getPosition() {
        return this.rotationsToMeters(this.rightMotor.getPosition().getValueAsDouble());
    }

    public void motionMagicPosition(double position, boolean clamp) {
        if (clamp) {
            position = MathUtil.clamp(position, ElevatorConstants.LOWER_HARD_STOP, ElevatorConstants.UPPER_HEIGHT_LIMIT);
        }

        MotionMagicVoltage control = motionMagicVoltage
            .withSlot(0)
            .withPosition(this.metersToRotation(position));   

        this.rightMotor.setControl(control
            .withLimitReverseMotion(this.bottomLimit.get())
            .withLimitForwardMotion(this.upperLimit.get()));
    }
    
    private double rotationsToMeters(double rotations) {
        return Math.PI * ElevatorConstants.ROLLER_DIAMETER * rotations;
    }

    private double metersToRotation(double meters) {
        return meters / ElevatorConstants.ROLLER_DIAMETER / Math.PI;
    }
    // getposition, setposition, motionMagicPosition, configureMotors (init), shuffleboard shit numberboard for position 0-1 meter we dont know height
    // basically all of it
    // set frequencey to 50hz

    // command has isFinished and sets motionMagicPosition
    // tolerance 1 cm
}
