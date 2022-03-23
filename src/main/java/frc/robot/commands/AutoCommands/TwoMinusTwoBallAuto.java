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

//score 2 balls in auto while placing 2 in the hangar

public class TwoMinusTwoBallAuto extends SequentialCommandGroup {
    public TwoMinusTwoBallAuto(DriveBase driveBase, Intake intake, Shooter shooter, Conveyor conveyor, Limelight limelight) {
        addRequirements(driveBase, intake, shooter, conveyor, limelight);

        PathPlannerTrajectory twoMinusTwoBallTrajectory1 = PathPlanner.loadPath("2m2path1",  
        0.8 * Constants.DriveBaseConstants.MAX_VELOCITY_METERS_PER_SECOND,
        1.0 * Constants.DriveBaseConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED, false);

        PathPlannerTrajectory twoMinusTwoBallTrajectory2 = PathPlanner.loadPath("2m2path2",  
        0.8 * Constants.DriveBaseConstants.MAX_VELOCITY_METERS_PER_SECOND,
        1.0 * Constants.DriveBaseConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED, false);

        PathPlannerTrajectory twoMinusTwoBallTrajectory3 = PathPlanner.loadPath("2m2path3",  
        0.8 * Constants.DriveBaseConstants.MAX_VELOCITY_METERS_PER_SECOND,
        1.0 * Constants.DriveBaseConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED, false);


        final Pose2d initial = new Pose2d(
            twoMinusTwoBallTrajectory1.getInitialPose().getTranslation(),
            ((PathPlannerState) twoMinusTwoBallTrajectory1.sample(0)).holonomicRotation);

        addCommands(
                new InstantCommand(() -> driveBase.resetOdometry(new Pose2d())),
                new InstantCommand(() -> intake.deploy()),
                new InstantCommand(() -> intake.setSpeed(1)),
                new SwerveControllerFollower(driveBase, twoMinusTwoBallTrajectory1).beforeStarting(() -> driveBase.resetOdometry(initial)),
                new InstantCommand(() -> intake.setSpeed(0)),
                new ShootCargo(driveBase, shooter, conveyor, limelight),
                new InstantCommand(() -> intake.setSpeed(1)),
                new SwerveControllerFollower(driveBase, twoMinusTwoBallTrajectory2),
                new InstantCommand(() -> intake.setSpeed(-.5)),
                new InstantCommand(() -> conveyor.setSpeed(-.4)),
                new WaitCommand(2),
                new InstantCommand(() -> conveyor.setSpeed(0)),
                new InstantCommand(() -> intake.setSpeed(0)),
                new InstantCommand(() -> intake.unDeploy()),
                new SwerveControllerFollower(driveBase, twoMinusTwoBallTrajectory3),
                new InstantCommand(() -> driveBase.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))));


    }
}
