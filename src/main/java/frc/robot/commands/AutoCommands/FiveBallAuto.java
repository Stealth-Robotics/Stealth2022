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

//score 5 balls in auto

public class FiveBallAuto extends SequentialCommandGroup {

    public FiveBallAuto(DriveBase driveBase, Intake intake, Shooter shooter, Conveyor conveyor,
            Limelight limelight) {

        addCommands(
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> intake.deploy()),
                                new InstantCommand(() -> intake.setSpeed(1)),
                                new SwerveControllerFollower(driveBase, "5BallPath1",
                                        Constants.DriveBaseConstants.AUTO_SPEED_CONFIG, false, true),
                                new InstantCommand(() -> intake.setSpeed(0)),
                                new ShootCargo(driveBase, shooter, conveyor,
                                        limelight, false),
                                new InstantCommand(() -> intake.setSpeed(1)),
                                new SwerveControllerFollower(driveBase, "5ballpath2",
                                        Constants.DriveBaseConstants.AUTO_SPEED_CONFIG, false, false),
                                new InstantCommand(() -> intake.setSpeed(0)),
                                new ShootCargo(driveBase, shooter, conveyor,
                                        limelight, false),
                                new InstantCommand(() -> intake.setSpeed(1)),
                                new SwerveControllerFollower(driveBase, "5ballpath3",
                                        Constants.DriveBaseConstants.AUTO_SPEED_CONFIG, false, false),
                                new WaitCommand(1),
                                new SwerveControllerFollower(driveBase, "5ballpath4",
                                        Constants.DriveBaseConstants.AUTO_SPEED_CONFIG, false, false),
                                new InstantCommand(() -> intake.setSpeed(0)),
                                new InstantCommand(() -> intake.unDeploy()),
                                new ShootCargo(driveBase, shooter, conveyor, limelight, false)),
                        new ConveyorDefault(conveyor, () -> false)));

        addRequirements(driveBase, intake, shooter, conveyor, limelight);

    }
}
