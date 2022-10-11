package frc.robot.commands.AutoCommands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

public class TwoBallAuto extends SequentialCommandGroup {

    public TwoBallAuto(DriveBase driveBase, Intake intake, Shooter shooter, Conveyor conveyor,
            Limelight limelight) {

        addCommands(
                new InstantCommand(
                        () -> driveBase.resetOdometry(new Pose2d())),
                new WaitCommand(3),
                new InstantCommand(() -> intake.deploy()),
                new InstantCommand(() -> intake.setSpeed(1)),
                new SwerveControllerFollower(driveBase,
                        TrajectoryGenerator.generateTrajectory(
                                new Pose2d(), List.of(),
                                new Pose2d(2, 0,
                                        new Rotation2d(0)),
                                Constants.DriveBaseConstants.SLOW_SPEED_CONFIG))
                                        .deadlineWith(new ConveyorDefault(conveyor, () -> false)),
                new ShootCargo(driveBase, shooter, conveyor,
                        limelight, true, 0.0),
                new InstantCommand(() -> driveBase.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))));

        addRequirements(driveBase, intake, shooter, conveyor, limelight);
    }
}
