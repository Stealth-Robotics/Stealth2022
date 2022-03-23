package frc.robot.commands.AutoCommands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

public class TwoMinusOneBallAuto extends SequentialCommandGroup {
    public TwoMinusOneBallAuto(DriveBase driveBase, Intake intake, Shooter shooter, Conveyor conveyor,
            Limelight limelight) {
        addRequirements(driveBase, intake, shooter, conveyor, limelight);

        PathPlannerTrajectory twoMinusOneBallTrajectory1 = PathPlanner.loadPath("2m1path1",
                0.8 * Constants.DriveBaseConstants.MAX_VELOCITY_METERS_PER_SECOND,
                0.1 * Constants.DriveBaseConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED, false);

        PathPlannerTrajectory twoMinusOneBallTrajectory2 = PathPlanner.loadPath("2m1path2",
                0.8 * Constants.DriveBaseConstants.MAX_VELOCITY_METERS_PER_SECOND,
                0.1 * Constants.DriveBaseConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED, false);

        PathPlannerTrajectory twoMinusOneBallTrajectory3 = PathPlanner.loadPath("2m1path3",
                0.8 * Constants.DriveBaseConstants.MAX_VELOCITY_METERS_PER_SECOND,
                0.1 * Constants.DriveBaseConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED, false);

        final Pose2d initial = new Pose2d(
                twoMinusOneBallTrajectory1.getInitialPose().getTranslation(),
                ((PathPlannerState) twoMinusOneBallTrajectory1.sample(0)).holonomicRotation);

        addCommands(
                new InstantCommand(() -> driveBase.resetOdometry(new Pose2d())),
                new InstantCommand(() -> intake.deploy()),
                new InstantCommand(() -> intake.setSpeed(1)),
                new SwerveControllerFollower(driveBase, twoMinusOneBallTrajectory1)
                        .beforeStarting(() -> driveBase.resetOdometry(initial)),
                new InstantCommand(() -> intake.setSpeed(0)),
                new ShootCargo(driveBase, shooter, conveyor, limelight),
                new InstantCommand(() -> intake.setSpeed(1)),
                new SwerveControllerFollower(driveBase, twoMinusOneBallTrajectory2),
                new InstantCommand(() -> intake.setSpeed(-.5)),
                new InstantCommand(() -> conveyor.setSpeed(-.4)),
                new WaitCommand(2),
                new InstantCommand(() -> conveyor.setSpeed(0)),
                new InstantCommand(() -> intake.setSpeed(0)),
                new InstantCommand(() -> intake.unDeploy()),
                new SwerveControllerFollower(driveBase, twoMinusOneBallTrajectory3),
                new InstantCommand(() -> driveBase.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))));

    }
}
