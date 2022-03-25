// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import org.ejml.dense.row.CovarianceOps_DDRM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoCommands.FiveBallAuto;
import frc.robot.commands.AutoCommands.TwoBallAuto;
import frc.robot.commands.AutoCommands.TwoMinusOneBallAuto;
import frc.robot.commands.AutoCommands.TwoMinusTwoBallAuto;
import frc.robot.commands.ClimberCommands.AutoClimb;
import frc.robot.commands.ClimberCommands.ClimberDefault;
import frc.robot.commands.ClimberCommands.MoveClimber;
import frc.robot.commands.ConveyerCommands.ConveyorDefault;
import frc.robot.commands.ConveyerCommands.MoveConveyor;
import frc.robot.commands.DriveBaseCommands.DriveDefault;
import frc.robot.commands.DriveBaseCommands.SwerveControllerFollower;
import frc.robot.commands.IntakeCommands.IntakeDefault;
import frc.robot.commands.MultiSubsystemCommands.EjectTopCargo;
import frc.robot.commands.MultiSubsystemCommands.ShootCargo;
import frc.robot.commands.MultiSubsystemCommands.ShootTopCargo;
import frc.robot.commands.ShooterCommands.ResetShooter;
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

        private final XboxController driveGamepad = new XboxController(Constants.IOConstants.DRIVE_JOYSTICK_PORT);
        private final XboxController mechGamepad = new XboxController(Constants.IOConstants.MECH_GAMEPAD_PORT);
        // private final Joystick driverStation = new
        // Joystick(Constants.IOConstants.DRIVER_STATION_PORT);

        SendableChooser<Command> autoChooser;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {

                driveBase.setDefaultCommand(new DriveDefault(
                                driveBase,
                                () -> -driveGamepad.getLeftY(),
                                () -> -driveGamepad.getLeftX(),
                                () -> -driveGamepad.getRightX(),
                                () -> driveGamepad.getRightBumper()));

                intake.setDefaultCommand(new IntakeDefault(intake,
                                driveGamepad::getRightTriggerAxis, driveGamepad::getLeftTriggerAxis));

                // TODO: Check And Set Override Button
                conveyor.setDefaultCommand(new ConveyorDefault(conveyor, () -> driveGamepad.getStartButton()));
                climber.setDefaultCommand(new ClimberDefault(climber, () -> mechGamepad.getRawAxis(3),
                                () -> mechGamepad.getRawAxis(4), () -> mechGamepad.getRawButton(6)));

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

                // TODO: Check Button Numbers
                new JoystickButton(mechGamepad, 4).whenPressed(new ShootTopCargo(shooter, conveyor, limelight));
                new JoystickButton(mechGamepad, 2).whenPressed(new EjectTopCargo(shooter, conveyor));
                new JoystickButton(mechGamepad, 5).whenPressed(new InstantCommand(() -> climber.movePisitons(true)));
                new JoystickButton(mechGamepad, 6).whenPressed(new InstantCommand(() -> climber.movePisitons(false)));

                new JoystickButton(mechGamepad, 10).whenPressed(new MoveClimber(climber, 80000));
                // new JoystickButton(mechGamepad, 10).whenPressed(new AutoClimb(climber));

                // new JoystickButton(driveGamepad, 2).whenPressed(() ->
                // driveBase.resetOdometry(new Pose2d()));
                new JoystickButton(driveGamepad, 2).whenPressed(
                                new SequentialCommandGroup(
                                                new MoveConveyor(conveyor, -500),
                                                new InstantCommand(() -> shooter.setSpeed(.3)),
                                                new InstantCommand(() -> shooter.hoodToDegree(72)),
                                                new MoveConveyor(conveyor,
                                                                Constants.ConveyorConstants.SHOOT_CONVEYOR_STEP * 2),
                                                new ResetShooter(shooter),
                                                new InstantCommand(() -> driveBase.drive(0, 0, 0)),
                                                new InstantCommand(() -> shooter.setVelocity(0)),
                                                new InstantCommand(() -> shooter.hoodToPos(0))));

                autoChooser = new SendableChooser<>();
                autoChooser.setDefaultOption("Two Ball Auto",
                                new TwoBallAuto(driveBase, intake, shooter, conveyor, limelight));
                autoChooser.addOption("Five Ball Auto",
                                new FiveBallAuto(driveBase, intake, shooter, conveyor, limelight));
                autoChooser.addOption("TwoBall_MinusOne",
                                new TwoMinusOneBallAuto(driveBase, intake, shooter, conveyor, limelight));
                autoChooser.addOption("TwoBall_MinusTwo",
                                new TwoMinusTwoBallAuto(driveBase, intake, shooter, conveyor, limelight));

                SmartDashboard.putData("Selected Autonomous", autoChooser);
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {

                // return new InstantCommand();

                // return new TwoBallAuto(driveBase, intake, shooter, conveyor, limelight);

                // return new FiveBallAuto(driveBase, intake, shooter, conveyor, limelight);
                return autoChooser.getSelected();
        }
}
