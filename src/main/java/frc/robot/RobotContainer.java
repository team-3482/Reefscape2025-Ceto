// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.algae.AlgaeSubsystem;
import frc.robot.auto.PIDAlignReefCommand;
import frc.robot.constants.Constants.ControllerConstants;
import frc.robot.constants.Constants.ScoringConstants;
import frc.robot.coral.AdjustCoralCommand;
import frc.robot.coral.CoralSubsystem;
import frc.robot.coral.IntakeCoralCommand;
import frc.robot.coral.OuttakeCoralCommand;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.elevator.MoveElevatorCommand;
import frc.robot.elevator.ZeroElevatorCommand;
import frc.robot.led.LEDSubsystem;
import frc.robot.swerve.OscillateXDirectionCommand;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.swerve.SwerveTelemetry;
import frc.robot.swerve.TunerConstants;
import frc.robot.utilities.CommandGenerators;
import frc.robot.vision.VisionSubsystem;

import java.util.Map;
import java.util.function.Supplier;

public class RobotContainer {
    // Use Bill Pugh Singleton Pattern for efficient lazy initialization (thread-safe !)
    private static class RobotContainerHolder {
        private static final RobotContainer INSTANCE = new RobotContainer();
    }
    
    public static RobotContainer getInstance() {
        return RobotContainerHolder.INSTANCE;
    }

    private final SendableChooser<Command> autoChooser;
    
    // Instance of the controllers used to drive the robot
    private final CommandXboxController driverController;
    private final CommandXboxController operatorController;
    private final XboxController driverController_HID;
    private final XboxController operatorController_HID;

    private Command auton = null;

    public RobotContainer() {
        this.driverController = new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_ID);
        this.operatorController = new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_ID);
        this.driverController_HID = this.driverController.getHID();
        this.operatorController_HID = this.operatorController.getHID();

        configureDrivetrain(); // This is done separately because it works differently from other Subsystems
        initializeSubsystems();

        // Register named commands for Pathplanner (always do this after subsystem initialization)
        registerNamedCommands();

        this.autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be Commands.none()
        this.autoChooser.onChange((Command autoCommand) -> this.auton = autoCommand); // Reloads the stored auto
        
        SmartDashboard.putData("Auto Chooser", this.autoChooser);
        SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
    }

    /**
     * This function initializes the swerve subsystem and configures its bindings with the driver controller.
     * This is based on the {@code Phoenix6 Swerve Example} that can be found on GitHub.
     */
    private void configureDrivetrain() {
        final SwerveSubsystem Drivetrain = SwerveSubsystem.getInstance();
        final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond);
        final double LesserMaxSpeed = TunerConstants.kTopSpeed.in(Units.MetersPerSecond);
        final double NormalAngularSpeed = TunerConstants.kAngularSpeedNormal.in(Units.RadiansPerSecond);
        final double FastAngularSpeed = TunerConstants.kAngularSpeedFast.in(Units.RadiansPerSecond);

        final SwerveRequest.FieldCentric fieldCentricDrive_withDeadband = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        
        final SwerveTelemetry logger = new SwerveTelemetry(MaxSpeed);

        Supplier<Boolean> leftTrigger = () -> this.driverController_HID.getLeftTriggerAxis() >= 0.5;
        Supplier<Boolean> rightTrigger = () -> this.driverController_HID.getRightTriggerAxis() >= 0.5;

        // Drivetrain will execute this command periodically
        Drivetrain.setDefaultCommand(
            Drivetrain.applyRequest(() -> {
                boolean elevatorTooHigh = ElevatorSubsystem.getInstance().getPosition() > ScoringConstants.SLOW_DRIVE_HEIGHT;
                boolean topSpeed = leftTrigger.get();
                boolean fineControl = rightTrigger.get();

                double linearSpeed = elevatorTooHigh
                    ? (TunerConstants.kElevatorTooHighSpeed.in(Units.MetersPerSecond))
                    : (topSpeed ? MaxSpeed : LesserMaxSpeed);
                double angularSpeed = elevatorTooHigh
                    ? (TunerConstants.kElevatorTooHighAngularSpeed.in(Units.MetersPerSecond))
                    : (topSpeed ? NormalAngularSpeed : FastAngularSpeed);
                double fineControlMult = fineControl ? ControllerConstants.FINE_CONTROL_MULT : 1;
                
                return fieldCentricDrive_withDeadband
                    // Drive forward with negative Y (forward)
                    .withVelocityX(-driverController.getLeftY() * linearSpeed * fineControlMult)
                    // Drive left with negative X (left)
                    .withVelocityY(-driverController.getLeftX() * linearSpeed * fineControlMult)
                    // Drive counterclockwise with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * angularSpeed * fineControlMult)

                    .withDeadband(ControllerConstants.DEADBAND * linearSpeed)
                    .withRotationalDeadband(ControllerConstants.DEADBAND * angularSpeed);
            }).ignoringDisable(true)
        );
        
        // Useful for testing
        // final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
        // this.driverController.y().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(
        //     new Rotation2d(
        //         Math.abs(driverController.getLeftY()) >= 0.25 ? -driverController.getLeftY() : 0,
        //         Math.abs(driverController.getLeftX()) >= 0.25 ? -driverController.getLeftX() : 0
        //     )
        // )));

        // POV / D-PAD
        final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        
        // This looks terrible, but I can't think of a better way to do it </3
        if (ControllerConstants.DPAD_DRIVE_INPUT) {
            /** POV angle : [X velocity, Y velocity] in m/s */
            final Map<Integer, Integer[]> povSpeeds = Map.ofEntries(
                Map.entry(  0, new Integer[]{ 1,  0}),
                Map.entry( 45, new Integer[]{ 1, -1}),
                Map.entry( 90, new Integer[]{ 0, -1}),
                Map.entry(135, new Integer[]{-1, -1}),
                Map.entry(180, new Integer[]{-1,  0}),
                Map.entry(225, new Integer[]{-1,  1}),
                Map.entry(270, new Integer[]{ 0,  1}),
                Map.entry(315, new Integer[]{ 1,  1})
            );

            povSpeeds.forEach(
                (Integer angle, Integer[] speeds) -> this.driverController.pov(angle).whileTrue(
                    Drivetrain.applyRequest(() -> {
                        boolean faster = leftTrigger.get();
                        boolean robotCentric = rightTrigger.get();
                        
                        return robotCentric
                            ? robotCentricDrive
                                .withVelocityX(speeds[0] * (faster ? 1.5 : 0.25))
                                .withVelocityY(speeds[1] * (faster ? 1.5 : 0.25))
                            : fieldCentricDrive
                                .withVelocityX(speeds[0] * (faster ? 1.5 : 0.25))
                                .withVelocityY(speeds[1] * (faster ? 1.5 : 0.25));
                    })
                )
            );
        }
        
        Drivetrain.registerTelemetry(logger::telemeterize);
    }

    /** Creates instances of each subsystem so periodic runs on startup. */
    private void initializeSubsystems() {
        LEDSubsystem.getInstance();
        VisionSubsystem.getInstance();
        
        ElevatorSubsystem.getInstance();
        CoralSubsystem.getInstance();
        AlgaeSubsystem.getInstance();
    }

    /** Register all NamedCommands for PathPlanner use */
    private void registerNamedCommands() {
        NamedCommands.registerCommand("MoveElevatorToBottom",
            new MoveElevatorCommand(ScoringConstants.INTAKING_HEIGHT, false, false));
        NamedCommands.registerCommand("MoveElevatorToIdle",
            new MoveElevatorCommand(ScoringConstants.IDLE_HEIGHT, false, false));

        NamedCommands.registerCommand("MoveElevatorToL1Coral",
            new MoveElevatorCommand(ScoringConstants.L1_CORAL, false, false));
        NamedCommands.registerCommand("MoveElevatorToL2Coral",
            new MoveElevatorCommand(ScoringConstants.L2_CORAL, false, false));
        NamedCommands.registerCommand("MoveElevatorToL3Coral",
            new MoveElevatorCommand(ScoringConstants.L3_CORAL, false, false));

        NamedCommands.registerCommand("MoveElevatorToL2Algae",
            new MoveElevatorCommand(ScoringConstants.L2_ALGAE, false, false));
        NamedCommands.registerCommand("MoveElevatorToL3Algae",
            new MoveElevatorCommand(ScoringConstants.L3_ALGAE, false, false));
        
        // NamedCommands.registerCommand("MoveElevatorToBottom_Slow",
        //     new MoveElevatorCommand(ScoringConstants.BOTTOM_HEIGHT, true, false));
        // NamedCommands.registerCommand("MoveElevatorToL1Coral_Slow",
        //     new MoveElevatorCommand(ScoringConstants.L1_CORAL, true, false));
        // NamedCommands.registerCommand("MoveElevatorToL2Coral_Slow",
        //     new MoveElevatorCommand(ScoringConstants.L2_CORAL, true, false));
        // NamedCommands.registerCommand("MoveElevatorToL3Coral_Slow",
        //     new MoveElevatorCommand(ScoringConstants.L3_CORAL, true, false));
        // NamedCommands.registerCommand("MoveElevatorToL2Algae_Slow",
        //     new MoveElevatorCommand(ScoringConstants.L2_ALGAE, true, false));
        
        NamedCommands.registerCommand("IntakeCoral",
            new IntakeCoralCommand());
        NamedCommands.registerCommand("OuttakeCoral",
            new OuttakeCoralCommand());
        NamedCommands.registerCommand("AdjustCoral",
            new AdjustCoralCommand());
        NamedCommands.registerCommand("EnableAlgae",
            CommandGenerators.EnableAlgaeCommand());
        NamedCommands.registerCommand("DisableAlgae",
            CommandGenerators.DisableAlgaeCommand());
        
        NamedCommands.registerCommand("PIDAlignRightReef",
            new PIDAlignReefCommand(1, false, false, true)
        );
        NamedCommands.registerCommand("PIDAlignLeftReef",
            new PIDAlignReefCommand(-1, false, false, true)
        );
        NamedCommands.registerCommand("PIDAlignCenterReef",
            new PIDAlignReefCommand(0, false, false, true)
        );
    }

    /** Configures the button bindings of the driver controller. */
    public void configureDriverBindings() {
        this.driverController.b().onTrue(CommandGenerators.CancelAllCommands());

         /*
         * POV, joysticks, and start/back are all used in configureDrivetrain()
         *           Left joystick : Translational movement
         *          Right joystick : Rotational movement
         *    POV (overrides joys) : Directional movement -- 0.25 m/s
         */
        // Burger
        this.driverController.start().onTrue(CommandGenerators.SetForwardDirectionCommand());
        // Double Rectangle
        this.driverController.back().onTrue(CommandGenerators.ResetOdometryToOriginCommand());

        /*
         * Triggers are also used in configureDrivetrain()
         *      Left Trigger > 0.5 : Use TOP SPEED for joysticks
         *                           Use 1.5 m/s for POV
         *     Right Trigger > 0.5 : Use FINE CONTROL for joysticks
         *                           Use ROBOT CENTRIC for POV 
         */
        // this.driverController.x().whileTrue(
        //     SwerveSubsystem.getInstance().applyRequest(() -> new SwerveRequest.SwerveDriveBrake())
        // );
        this.driverController.x().whileTrue(new OscillateXDirectionCommand());

        this.driverController.leftBumper().whileTrue(Commands.sequence(
            new PIDAlignReefCommand(-1, true, true, true),
            new PIDAlignReefCommand(-1, true, false, false)
        ));
        this.driverController.rightBumper().whileTrue(Commands.sequence(
            new PIDAlignReefCommand(1, true, true, true),
            new PIDAlignReefCommand(1, true, false, false)
        ));
        this.driverController.a().whileTrue(new PIDAlignReefCommand(0, false, false, true));
    }

    /** Configures the button bindings of the operator controller. */
    public void configureOperatorBindings() {
        this.operatorController.b().onTrue(CommandGenerators.CancelAllCommands());

        Supplier<Boolean> slowElevatorSupplier = () -> this.operatorController_HID.getRightTriggerAxis() >= 0.5;

        // Elevator
        this.operatorController.leftTrigger()
            .onTrue(new MoveElevatorCommand(ScoringConstants.INTAKING_HEIGHT, slowElevatorSupplier, false));
        this.operatorController.povLeft()
            .onTrue(new ZeroElevatorCommand());

        this.operatorController.povDown()
            .toggleOnTrue(new MoveElevatorCommand(ScoringConstants.L1_CORAL, slowElevatorSupplier, true));
        this.operatorController.povRight()
            .toggleOnTrue(new MoveElevatorCommand(ScoringConstants.L2_CORAL, slowElevatorSupplier, true));
        this.operatorController.povUp()
            .toggleOnTrue(new MoveElevatorCommand(ScoringConstants.L3_CORAL, slowElevatorSupplier, true));

        this.operatorController.a()
            .toggleOnTrue(new MoveElevatorCommand(ScoringConstants.L2_ALGAE, slowElevatorSupplier, true))
            .onTrue(CommandGenerators.EnableAlgaeCommand());
        this.operatorController.y()
            .toggleOnTrue(new MoveElevatorCommand(ScoringConstants.L3_ALGAE, slowElevatorSupplier, true))
            .onTrue(CommandGenerators.EnableAlgaeCommand());
        
        // Algae
        this.operatorController.x()
            .onTrue(CommandGenerators.EnableAlgaeCommand())
            .onFalse(CommandGenerators.DisableAlgaeCommand());

        // Coral
        this.operatorController.leftBumper()
            .onTrue(CommandGenerators.DisableAlgaeCommand())
            .whileTrue(
                Commands.parallel(
                    new MoveElevatorCommand(ScoringConstants.INTAKING_HEIGHT, slowElevatorSupplier, true),
                    new IntakeCoralCommand()
                )
            );

        final Timer timer = new Timer();
        
        this.operatorController.rightBumper()
            .onTrue(Commands.runOnce(() -> timer.restart()))

            .whileTrue(new OuttakeCoralCommand())
            
            .onFalse(Commands.sequence(
                new MoveElevatorCommand(Double.NaN, false, false),
                Commands.waitUntil(() -> timer.hasElapsed(1)),
                new MoveElevatorCommand(ScoringConstants.IDLE_HEIGHT, false, false)
                    // Only move elevator down if we aren't intaking
                    .onlyIf(() -> ElevatorSubsystem.getInstance().getLastSetGoal() != ScoringConstants.INTAKING_HEIGHT),
                CommandGenerators.DisableAlgaeCommand()
            ));

        this.operatorController.rightStick().onTrue(new AdjustCoralCommand());
    }

    /**
     * Gets the instance of the driverController.
     * @return The driver controller.
     */
    public CommandXboxController getDriverController() {
        return this.driverController;
    }

    /**
     * Gets the instance of the operatorController.
     * @return The operator controller.
     */
    public CommandXboxController getOperatorController() {
        return this.operatorController;
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * @return The command to run in autonomous.
     */
    public Command getAutonomousCommand() {
        if (this.auton == null) {
            this.auton = this.autoChooser.getSelected();
        }
        return this.auton;
    }
}
