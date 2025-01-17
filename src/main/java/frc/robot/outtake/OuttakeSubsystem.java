// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.outtake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PhysicalConstants;

public class OuttakeSubsystem extends SubsystemBase {
    // Thread-safe singleton design pattern.
    private static volatile OuttakeSubsystem instance;
    private static Object mutex = new Object();

    private TalonFX rightMotor = new TalonFX(PhysicalConstants.OuttakeConstants.RIGHT_MOTOR_ID);
    private TalonFX leftMotor = new TalonFX(PhysicalConstants.OuttakeConstants.LEFT_MOTOR_ID); 
    private DigitalInput frontLaser = new DigitalInput(PhysicalConstants.OuttakeConstants.FRONT_LASER_ID);


    public static OuttakeSubsystem getInstance() {
        OuttakeSubsystem result = instance;
        
        if (result == null) {
            synchronized (mutex) {
                result = instance;
                if (result == null) {
                    instance = result = new OuttakeSubsystem();
                }
            }
        }
        return instance;
    }

    /** Creates a new ExampleSubsystem. */
    private OuttakeSubsystem() {
        super("OuttakeSubsystem");
        configureMotors();

        Follower follow = new Follower(PhysicalConstants.OuttakeConstants.RIGHT_MOTOR_ID, true);
        this.leftMotor.setControl(follow);
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {}

    /**
     * Sets the voltage to v
     * @param v, the voltage you want to set
     */
    private void voltage(double v)
    {
        rightMotor.setVoltage(v);
    }
    /**
     * Sets the speed of the motors to the intake speed
     */
    public void intake()
    {
        rightMotor.setVoltage(PhysicalConstants.OuttakeConstants.INTAKE_VOLTAGE);
    }
    /**
     * Sets the speed of the motors to the outtake speed
     */
    public void outtake()
    {
        rightMotor.setVoltage(PhysicalConstants.OuttakeConstants.OUTTAKE_VOLTAGE);
    }
    /**
     * Configures the gear ratio of the motors to our gear ratio
     */
    public void configureMotors()
    {
        TalonFXConfiguration configuration = new TalonFXConfiguration();
        configuration.Feedback.SensorToMechanismRatio = PhysicalConstants.OuttakeConstants.MECHANISM_RATIO;
    }
    /**
     * Stops the motors immediately
     */
    public void stop()
    {
        rightMotor.setVoltage(0);
    }
    /**
     * Returns a boolean value of whether there is a coral in the shooter
     * @return whether it has coral
     */
    public boolean hasCoral()
    {
        return !frontLaser.get();
    }
}
