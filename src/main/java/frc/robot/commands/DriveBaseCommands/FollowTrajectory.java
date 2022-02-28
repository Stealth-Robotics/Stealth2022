package frc.robot.commands.DriveBaseCommands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveBase;

public class FollowTrajectory extends SequentialCommandGroup {

    public FollowTrajectory(DriveBase driveBase, Trajectory trajectory) {

        addRequirements(driveBase);

        addCommands(
                driveBase.getSwerveControllerCommand(trajectory),
                new InstantCommand(() -> driveBase.drive(0.0, 0.0, 0.0)));
    }
}
