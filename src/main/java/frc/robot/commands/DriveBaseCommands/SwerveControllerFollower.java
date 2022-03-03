package frc.robot.commands.DriveBaseCommands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;

public class SwerveControllerFollower extends CommandBase {
    private final DriveBase driveBase;
    private final Trajectory trajectory;
    private final Timer timer = new Timer();

    public SwerveControllerFollower(DriveBase driveBase, Trajectory trajectory) {
        this.driveBase = driveBase;
        this.trajectory = trajectory;

        addRequirements(driveBase);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        driveBase.resetPathController();
    }

    @Override
    public void execute() {
        double currentTime = timer.get();
        Trajectory.State targetState = trajectory.sample(currentTime);
        Rotation2d targetRotation = targetState.poseMeters.getRotation();
        if (trajectory instanceof PathPlannerTrajectory) {
            targetState = ((PathPlannerTrajectory) trajectory).sample(currentTime);
            targetRotation = ((PathPlannerState) targetState).holonomicRotation;
        }

        driveBase.drive(targetState, targetRotation);

        Pose2d targetPose = targetState.poseMeters;
        
        System.out.println("Target X: " + targetPose.getX());
        System.out.println("Target Y: " + targetPose.getY());
        System.out.println("Target Heading" + targetPose.getRotation().getDegrees());
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }
}
