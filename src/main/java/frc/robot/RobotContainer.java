// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoCommands.DoNothingAuto;
import frc.robot.commands.AutoCommands.FiveBallAuto;
import frc.robot.commands.AutoCommands.TwoBallAuto;
import frc.robot.commands.AutoCommands.TwoMinusOneBallAuto;
import frc.robot.commands.AutoCommands.TwoMinusTwoBallAuto;
import frc.robot.commands.ClimberCommands.AutoClimb;
import frc.robot.commands.ClimberCommands.ClimberDefault;
import frc.robot.commands.ClimberCommands.MoveClimber;
import frc.robot.commands.ConveyerCommands.ConveyorDefault;
import frc.robot.commands.DriveBaseCommands.DriveDefault;
import frc.robot.commands.IntakeCommands.IntakeDefault;
import frc.robot.commands.MultiSubsystemCommands.EjectTopCargo;
import frc.robot.commands.MultiSubsystemCommands.ShootCargo;
import frc.robot.commands.MultiSubsystemCommands.ShootCargoLow;
import frc.robot.commands.MultiSubsystemCommands.ShootTopCargo;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class RobotContainer {

    private final DriveBase driveBase = new DriveBase();
    private final Intake intake = new Intake();
    private final Shooter shooter = new Shooter();
    private final Conveyor conveyor = new Conveyor();
    private final Limelight limelight = new Limelight();
    private final Climber climber = new Climber();
    // private final CANdleSystem candle = new CANdleSystem();

    private final XboxController driveGamepad = new XboxController(Constants.IOConstants.DRIVE_JOYSTICK_PORT);
    private final XboxController mechGamepad = new XboxController(Constants.IOConstants.MECH_GAMEPAD_PORT);
    // private final Joystick driverStation = new
    // Joystick(Constants.IOConstants.DRIVER_STATION_PORT);

    SendableChooser<Command> autoChooser = new SendableChooser<>();
    UsbCamera camera = CameraServer.startAutomaticCapture();

    public RobotContainer() {
        autoChooser.setDefaultOption("Five Ball Auto",
                new FiveBallAuto(driveBase, intake, shooter, conveyor, limelight));
        autoChooser.addOption("Two Ball Auto",
                new TwoBallAuto(driveBase, intake, shooter, conveyor, limelight));
        autoChooser.addOption("Two Ball Minus One",
                new TwoMinusOneBallAuto(driveBase, intake, shooter, conveyor, limelight));
        autoChooser.addOption("Two Ball Minus Two",
                new TwoMinusTwoBallAuto(driveBase, intake, shooter, conveyor, limelight));
        autoChooser.addOption("Do nothing",
                new DoNothingAuto());

        SmartDashboard.putData("Selected Autonomous", autoChooser);

        camera.setResolution(160, 120);
        camera.setFPS(30);

        driveBase.setDefaultCommand(new DriveDefault(
                driveBase,
                () -> -driveGamepad.getLeftY(),
                () -> -driveGamepad.getLeftX(),
                () -> -driveGamepad.getRightX(),
                () -> driveGamepad.getRightBumper()));

        intake.setDefaultCommand(new IntakeDefault(intake,
                driveGamepad::getRightTriggerAxis, driveGamepad::getLeftTriggerAxis,
                () -> DriverStation.isAutonomous()));

        // TODO: Check And Set Override Button
        conveyor.setDefaultCommand(new ConveyorDefault(conveyor, () -> driveGamepad.getStartButton()));
        climber.setDefaultCommand(new ClimberDefault(climber, () -> mechGamepad.getRawAxis(3),
                () -> mechGamepad.getRawAxis(4), () -> mechGamepad.getRawButton(6)));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // drive gamepad controls
        new JoystickButton(driveGamepad, 1)
                .whenPressed(() -> driveBase.resetOdometry(new Pose2d()));

        new JoystickButton(driveGamepad, 6)
                .whenPressed(new ShootCargo(driveBase, shooter, conveyor, limelight, false));

        // TODO: Check Button Numbers
        new JoystickButton(mechGamepad, 4).whenPressed(new ShootTopCargo(shooter, conveyor, limelight));
        new JoystickButton(mechGamepad, 2).whenPressed(new EjectTopCargo(shooter, conveyor));
        new JoystickButton(mechGamepad, 5).whenPressed(new InstantCommand(() -> climber.movePisitons(true)));
        new JoystickButton(mechGamepad, 6).whenPressed(new InstantCommand(() -> climber.movePisitons(false)));

        new JoystickButton(mechGamepad, 9).whenPressed(new MoveClimber(climber, 97500));
        new JoystickButton(mechGamepad, 10).whenPressed(new AutoClimb(climber));
        new JoystickButton(driveGamepad, 2).whenPressed(new ShootCargoLow(driveBase, shooter, conveyor, limelight));

    }

    public Command getAutonomousCommand() {

        return autoChooser.getSelected()
                .beforeStarting(new InstantCommand(() -> climber.resetClimberEncoder()));
    }
}
