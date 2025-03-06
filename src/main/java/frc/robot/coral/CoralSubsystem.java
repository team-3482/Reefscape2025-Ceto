// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.coral;

import java.util.Map;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.ShuffleboardTabNames;
import frc.robot.constants.Constants.StatusColors;
import frc.robot.constants.PhysicalConstants.CoralConstants;
import frc.robot.constants.PhysicalConstants.RobotConstants;
import frc.robot.led.LEDSubsystem;
import org.littletonrobotics.junction.Logger;

/** A subsystem that manipulates the coral game piece. */
public class CoralSubsystem extends SubsystemBase {
    // Thread-safe singleton design pattern.
    private static volatile CoralSubsystem instance;
    private static Object mutex = new Object();

    public static CoralSubsystem getInstance() {
        CoralSubsystem result = instance;
        
        if (result == null) {
            synchronized (mutex) {
                result = instance;
                if (result == null) {
                    instance = result = new CoralSubsystem();
                }
            }
        }
        return instance;
    }

    private TalonFX rightMotor = new TalonFX(CoralConstants.RIGHT_MOTOR_ID, RobotConstants.CTRE_CAN_BUS);
    private TalonFX leftMotor = new TalonFX(CoralConstants.LEFT_MOTOR_ID, RobotConstants.CTRE_CAN_BUS); 
    private DigitalInput frontLaser = new DigitalInput(CoralConstants.FRONT_LASER_ID);
    private DigitalInput backLaser = new DigitalInput(CoralConstants.BACK_LASER_ID);

    private final ShuffleboardLayout shuffleboardLayout = Shuffleboard.getTab(ShuffleboardTabNames.DEFAULT)
        .getLayout("CoralSubsystem", BuiltInLayouts.kGrid)
        .withProperties(Map.of("Number of columns", 1, "Number of rows", 2, "Label position", "TOP"))
        .withSize(5, 3)
        .withPosition(1, 5);
    private GenericEntry shuffleboard_entry_frontLaser = shuffleboardLayout
        .add("Front Laser", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("colorWhenFalse", "black", "colorWhenTrue", "white"))
        .withSize(5, 1)
        .withPosition(0, 0)
        .getEntry();
    private GenericEntry shuffleboard_entry_backLaser = shuffleboardLayout
        .add("Back Laser", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("colorWhenFalse", "black", "colorWhenTrue", "white"))
        .withSize(5, 1)
        .withPosition(0, 1)
        .getEntry();

    private String state = "stopped";

    /** Creates a new OuttakeSubsystem. */
    private CoralSubsystem() {
        super("CoralSubsystem");
        configureMotors();

        Follower follow = new Follower(CoralConstants.RIGHT_MOTOR_ID, true);
        this.leftMotor.setControl(follow);
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        boolean coralFront = hasCoral_frontLaser();
        boolean coralBack = hasCoral_backLaser();
        
        this.shuffleboard_entry_frontLaser.setBoolean(coralFront);
        this.shuffleboard_entry_backLaser.setBoolean(coralBack);
        
        boolean coral = coralFront || coralBack;
        
        Logger.recordOutput("Coral/FrontLaserHasCoral", coralFront);
        Logger.recordOutput("Coral/BackLaserHasCoral", coralBack);
        Logger.recordOutput("Coral/HasCoral", coral);
        Logger.recordOutput("Coral/State", state);

        if (coral) {
            LEDSubsystem.getInstance().setColor(StatusColors.CORAL);
        }
    }

    /**
     * Configures the gear ratio of the motors to our gear ratio
     */
    private void configureMotors() {
        TalonFXConfiguration configuration = new TalonFXConfiguration();
        configuration.Feedback.SensorToMechanismRatio = CoralConstants.MECHANISM_RATIO;

        configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        CurrentLimitsConfigs currentLimitsConfigs = configuration.CurrentLimits;
        currentLimitsConfigs.StatorCurrentLimitEnable = true;
        currentLimitsConfigs.StatorCurrentLimit = 40;
        currentLimitsConfigs.SupplyCurrentLimitEnable = true;
        currentLimitsConfigs.SupplyCurrentLimit = 40;
        currentLimitsConfigs.SupplyCurrentLowerTime = 0.75;
        currentLimitsConfigs.SupplyCurrentLowerLimit = 30;

        configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        this.rightMotor.getConfigurator().apply(configuration);
        configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        this.leftMotor.getConfigurator().apply(configuration);
    }

    /**
     * Sets the speed of the motors to the intake speed
     */
    public void intake() {
        rightMotor.setVoltage(CoralConstants.INTAKE_VOLTAGE);
        state = "intaking";
    }

    /**
     * Sets the speed of the motors to the slow intake speed
     */
    public void slowIntake() {
        this.rightMotor.setVoltage(CoralConstants.SLOW_INTAKE_VOLTAGE);
        state = "slow intaking";
    }

    /**
     * Sets the speed of the motors to the outtake speed
     */
    public void outtake() {
        rightMotor.setVoltage(CoralConstants.OUTTAKE_VOLTAGE);
        state = "outtaking";
    }

    /**
     * Stops the motors immediately
     */
    public void stop() {
        rightMotor.setVoltage(0);
        state = "stopped";
    }

    /**
     * Checks if the front laser sees the coral
     * @return Whether it sees the coral
     */
    public boolean hasCoral_frontLaser() {
        return !this.frontLaser.get();
    }

    /**
     * Checks if the back laser sees the coral
     * @return Whether it sees the coral
     */
    public boolean hasCoral_backLaser() {
        return !this.backLaser.get();
    }

    /**
     * Returns a boolean value of whether there is a coral in the outtake
     * @return whether it has coral
     */
    public boolean hasCoral() {
        return hasCoral_backLaser() || hasCoral_frontLaser();
    }
}
