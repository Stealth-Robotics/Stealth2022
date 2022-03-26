package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.ConveyerCommands.ConveyorDefault;
import frc.robot.commands.DriveBaseCommands.SwerveControllerFollower;
import frc.robot.commands.MultiSubsystemCommands.ShootCargo;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

//score 2 balls in auto while placing 2 in the hangar

public class TwoMinusTwoBallAuto extends SequentialCommandGroup {

    public TwoMinusTwoBallAuto(DriveBase driveBase, Intake intake, Shooter shooter, Conveyor conveyor,
            Limelight limelight) {

        addCommands(
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> intake.deploy()),
                                new InstantCommand(() -> intake.setSpeed(1)),
                                new SwerveControllerFollower(driveBase,
                                        "2m2path1",
                                        Constants.DriveBaseConstants.AUTO_SPEED_CONFIG,
                                        false,
                                        true),
                                new InstantCommand(() -> intake.setSpeed(0)),
                                new ShootCargo(driveBase, shooter, conveyor, limelight, false),
                                new InstantCommand(() -> intake.setSpeed(1)),
                                new SwerveControllerFollower(driveBase,
                                        "2m2path2",
                                        Constants.DriveBaseConstants.AUTO_SPEED_CONFIG,
                                        false,
                                        false),
                                new InstantCommand(() -> intake.setSpeed(-0.5)),
                                new InstantCommand(() -> conveyor.setSpeed(-0.4)),
                                new WaitCommand(2),
                                new InstantCommand(() -> conveyor.setSpeed(0)),
                                new InstantCommand(() -> intake.setSpeed(0)),
                                new InstantCommand(() -> intake.unDeploy()),
                                new SwerveControllerFollower(driveBase,
                                        "2m2path3",
                                        Constants.DriveBaseConstants.AUTO_SPEED_CONFIG,
                                        false,
                                        false)),
                        new ConveyorDefault(conveyor, () -> false)));

        addRequirements(driveBase, intake, shooter, conveyor, limelight);
    }
}
