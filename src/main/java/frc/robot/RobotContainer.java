// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ConveyerCommands.ConveyorDefault;
import frc.robot.commands.ConveyerCommands.MoveConveyor;
import frc.robot.commands.DriveBaseCommands.DriveDefault;
import frc.robot.commands.IntakeCommands.IntakeDefault;
import frc.robot.commands.ShooterCommands.ReadyShooter;
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

  XboxController driveGamepad = new XboxController(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    driveBase.setDefaultCommand(new DriveDefault(
        driveBase,
        () -> (driveGamepad.getLeftX() * Constants.DriveBase.MAX_VELOCITY_METERS_PER_SECOND),
        () -> -(driveGamepad.getLeftY() * Constants.DriveBase.MAX_VELOCITY_METERS_PER_SECOND),
        () -> (driveGamepad.getRightX() * Constants.DriveBase.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)));

    intake.setDefaultCommand(new IntakeDefault(intake,
        driveGamepad::getRightTriggerAxis, driveGamepad::getLeftTriggerAxis));

    conveyor.setDefaultCommand(new ConveyorDefault(conveyor));

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
    new JoystickButton(driveGamepad, 1)
        .whenPressed(() -> driveBase.zeroGyroscope());

    new JoystickButton(driveGamepad, 2).whenPressed(
        new SequentialCommandGroup(
            // new InstantCommand(() -> limelight.setLedMode(3)),
            // add align to targer and ready shooter in a parralel deadline
            new ReadyShooter(shooter, 12 /* limelight.getTargetDistance() */),
            new MoveConveyor(conveyor, Constants.Conveyor.SHOOT_CONVEYOR_STEP * 2),
            new ParallelCommandGroup(
                new RunCommand(() -> shooter.hoodToPos(0)),
                new RunCommand(() -> shooter.setVelocity(0)))));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand();
  }
}
