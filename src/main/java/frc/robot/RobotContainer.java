// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.ClimberCommands.ClimberDefault;
import frc.robot.commands.ConveyerCommands.ConveyorDefault;
import frc.robot.commands.DriveBaseCommands.DriveDefault;
import frc.robot.commands.DriveBaseCommands.SwerveControllerFollower;
import frc.robot.commands.IntakeCommands.IntakeDefault;
import frc.robot.commands.MultiSubsystemCommands.EjectTopCargo;
import frc.robot.commands.MultiSubsystemCommands.ShootCargo;
import frc.robot.commands.MultiSubsystemCommands.ShootTopCargo;
import frc.robot.commands.MultiSubsystemCommands.ShootTopCargo;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems and commands are defined here...
        private final DriveBase driveBase = new DriveBase();
        private final Intake intake = new Intake();
        private final Shooter shooter = new Shooter();
        private final Conveyor conveyor = new Conveyor();
        private final Limelight limelight = new Limelight();
        private final Climber climber = new Climber();
        // private final CANdleSystem candle = new CANdleSystem();

        private final UsbCamera intakeCamera;

        private final Joystick buttonPanel = new Joystick(Constants.IO.BUTTON_PANEL_PORT);
        private final XboxController mechGamepad = new XboxController(2);
        private final XboxController driveGamepad = new XboxController(Constants.IO.DRIVE_JOYSTICK_PORT);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {

                driveBase.setDefaultCommand(new DriveDefault(
                                driveBase,
                                () -> -driveGamepad.getLeftY(),
                                () -> driveGamepad.getLeftX(),
                                () -> -driveGamepad.getRightX(),
                                () -> driveGamepad.getRightBumper()));

                intake.setDefaultCommand(new IntakeDefault(intake,
                                driveGamepad::getRightTriggerAxis, driveGamepad::getLeftTriggerAxis));

                conveyor.setDefaultCommand(new ConveyorDefault(conveyor, () -> driveGamepad.getStartButton()));
                climber.setDefaultCommand(new ClimberDefault(climber,() -> mechGamepad.getRightTriggerAxis(), () -> mechGamepad.getRightTriggerAxis()));

                intakeCamera = CameraServer.startAutomaticCapture(0);
                intakeCamera.setResolution(1280, 720);
                intakeCamera.setFPS(25);
                // Configure the button bindings
                configureButtonBindings();
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link GenericHID} or one of its subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
         * it to a {@link
         * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         */
        private void configureButtonBindings() {
                // drive gamepad controls
                new JoystickButton(driveGamepad, 1)
                                .whenPressed(() -> driveBase.zeroGyroscope());

                new JoystickButton(driveGamepad, 6)
                                .whenPressed(new ShootCargo(driveBase, shooter, conveyor, limelight));

               
                new JoystickButton(driveGamepad, 4).whenPressed(new InstantCommand(() -> climber.togglePivotPistons()));

                new JoystickButton(driveGamepad, 2).whenPressed(new InstantCommand(() -> climber.togglePivotPistons()));

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {

                return new SequentialCommandGroup(
                                new InstantCommand(() -> driveBase.resetOdometry(new Pose2d())),
                                new SwerveControllerFollower(driveBase, TrajectoryGenerator.generateTrajectory(
                                                new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                                                List.of(),
                                                new Pose2d(Units.feetToMeters(50), 0, Rotation2d.fromDegrees(0)),
                                                Constants.DriveBase.SLOW_SPEED_CONFIG)));
        }
}
