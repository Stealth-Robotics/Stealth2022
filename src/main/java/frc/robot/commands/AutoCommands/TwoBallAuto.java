package frc.robot.commands.AutoCommands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.DriveBaseCommands.SwerveControllerFollower;
import frc.robot.commands.MultiSubsystemCommands.ShootCargo;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class TwoBallAuto extends SequentialCommandGroup {
    public TwoBallAuto(DriveBase driveBase, Intake intake, Shooter shooter, Conveyor conveyor, Limelight limelight) {
        addRequirements(driveBase, intake, shooter, conveyor, limelight);

        addCommands(
                new InstantCommand(() -> driveBase.resetOdometry(new Pose2d())),
                new InstantCommand(() -> intake.deploy()),
                new InstantCommand(() -> intake.setSpeed(1)),
                // new WaitCommand(5),
                new SwerveControllerFollower(driveBase,
                        TrajectoryGenerator.generateTrajectory(new Pose2d(), List.of(),
                                new Pose2d(1.5, 0, new Rotation2d(0)),
                                Constants.DriveBaseConstants.SLOW_SPEED_CONFIG)),
                new ShootCargo(driveBase, shooter, conveyor, limelight));
    }
}
