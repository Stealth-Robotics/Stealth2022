package frc.robot.commands.AutoCommands;

import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.DriveBaseCommands.SwerveControllerFollower;
import frc.robot.commands.MultiSubsystemCommands.ShootCargo;
import frc.robot.commands.MultiSubsystemCommands.ShootCargoNoHoodReset;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;



public class FiveBallAuto extends SequentialCommandGroup {

    // final Pose2d initial = (trajectory instanceof PathPlannerTrajectory) ? new Pose2d(
    //     trajectory.getInitialPose().getTranslation(),
    //     ((PathPlannerState) ((PathPlannerTrajectory) trajectory).sample(0)).holonomicRotation)
    //     : trajectory.getInitialPose();
        
    public FiveBallAuto(DriveBase driveBase, Intake intake, Shooter shooter, Conveyor conveyor, Limelight limelight) {
        addRequirements(driveBase, intake, shooter, conveyor, limelight);

        PathPlannerTrajectory fiveballtrajectory1 = PathPlanner.loadPath("5BallPath1",  
        0.3 * Constants.DriveBaseConstants.MAX_VELOCITY_METERS_PER_SECOND,
        0.3 * Constants.DriveBaseConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED, false);

        PathPlannerTrajectory fiveballtrajectory2 = PathPlanner.loadPath("5ballpath2",  
        0.3 * Constants.DriveBaseConstants.MAX_VELOCITY_METERS_PER_SECOND,
        0.3 * Constants.DriveBaseConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED, false);

        PathPlannerTrajectory fiveballtrajectory3 = PathPlanner.loadPath("5ballpath3",  
        0.3 * Constants.DriveBaseConstants.MAX_VELOCITY_METERS_PER_SECOND,
        0.3 * Constants.DriveBaseConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED, false);

        PathPlannerTrajectory fiveballtrajectory4 = PathPlanner.loadPath("5ballpath4",  
        0.3 * Constants.DriveBaseConstants.MAX_VELOCITY_METERS_PER_SECOND,
        0.3 * Constants.DriveBaseConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED, false);

        final Pose2d initial = new Pose2d(
            fiveballtrajectory1.getInitialPose().getTranslation(),
            ((PathPlannerState) fiveballtrajectory1.sample(0)).holonomicRotation);


        addCommands(
                //new InstantCommand(() -> driveBase.resetOdometry(initial))),
                new InstantCommand(() -> intake.deploy()),
                new InstantCommand(() -> intake.setSpeed(1)),
                new SwerveControllerFollower(driveBase, fiveballtrajectory1).beforeStarting(() -> driveBase.resetOdometry(initial)),
                new InstantCommand(() -> intake.setSpeed(0)),
                new ShootCargoNoHoodReset(driveBase, shooter, conveyor, limelight),
                new InstantCommand(() -> intake.setSpeed(1)),
                new SwerveControllerFollower(driveBase, fiveballtrajectory2),
                new InstantCommand(() -> intake.setSpeed(0)),
                new ShootCargoNoHoodReset(driveBase, shooter, conveyor, limelight),
                new InstantCommand(() -> intake.setSpeed(1)),
                new SwerveControllerFollower(driveBase, fiveballtrajectory3),
                new WaitCommand(1),
                new SwerveControllerFollower(driveBase, fiveballtrajectory4),
                new InstantCommand(() -> intake.setSpeed(0)),
                new InstantCommand(() -> intake.unDeploy()),
                new ShootCargo(driveBase, shooter, conveyor, limelight),
                new InstantCommand(() -> driveBase.resetOdometry(new Pose2d(1.5, 0, new Rotation2d(Math.toRadians(-152))))));


    }
}
