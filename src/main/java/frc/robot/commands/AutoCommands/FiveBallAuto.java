package frc.robot.commands.AutoCommands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.ConveyerCommands.ConveyorDefault;
import frc.robot.commands.DriveBaseCommands.SwerveControllerFollower;
import frc.robot.commands.IntakeCommands.DelayedIntakeOn;
import frc.robot.commands.MultiSubsystemCommands.ShootCargo;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

//score 5 balls in auto

public class FiveBallAuto extends SequentialCommandGroup {

    final static PathPlannerTrajectory fiveBallTrajectory1 = PathPlanner.loadPath("5BallPath1",
            0.8 * Constants.DriveBaseConstants.MAX_VELOCITY_METERS_PER_SECOND,
            1.0 * Constants.DriveBaseConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED, false);

    final static PathPlannerTrajectory fiveBallTrajectory2 = PathPlanner.loadPath("5ballpath2",
            0.8 * Constants.DriveBaseConstants.MAX_VELOCITY_METERS_PER_SECOND,
            1.0 * Constants.DriveBaseConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED, false);

    final static PathPlannerTrajectory fiveBallTrajectory3 = PathPlanner.loadPath("5ballpath3",
            1.0 * Constants.DriveBaseConstants.MAX_VELOCITY_METERS_PER_SECOND,
            1.0 * Constants.DriveBaseConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED, false);

    final static PathPlannerTrajectory fiveBallTrajectory4 = PathPlanner.loadPath("5ballpath4",
            0.4 * Constants.DriveBaseConstants.MAX_VELOCITY_METERS_PER_SECOND,
            1.0 * Constants.DriveBaseConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED, false);

    final static PathPlannerTrajectory fiveBallTrajectory5 = PathPlanner.loadPath("5ballpath5",
            1.0 * Constants.DriveBaseConstants.MAX_VELOCITY_METERS_PER_SECOND,
            1.0 * Constants.DriveBaseConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED, false);

    final Pose2d initial = new Pose2d(
            fiveBallTrajectory1.getInitialPose().getTranslation(),
            ((PathPlannerState) fiveBallTrajectory1.sample(0)).holonomicRotation);

    public FiveBallAuto(DriveBase driveBase, Intake intake, Shooter shooter, Conveyor conveyor,
            Limelight limelight) {

        addCommands(
                new InstantCommand(() -> intake.deploy()).beforeStarting(new InstantCommand(() -> driveBase
                        .resetOdometry(initial))),
                new InstantCommand(() -> intake.setSpeed(1)),
                new SwerveControllerFollower(driveBase, fiveBallTrajectory1)
                        .beforeStarting(new InstantCommand(() -> driveBase
                                .resetOdometry(initial)))
                        .deadlineWith(new ConveyorDefault(conveyor, () -> false)),
                new ShootCargo(driveBase, shooter, conveyor,
                        limelight, true, 106.0), //TODO: validate distance
                new InstantCommand(() -> intake.setSpeed(0)),
                new SwerveControllerFollower(driveBase, fiveBallTrajectory2).deadlineWith(
                        new ConveyorDefault(conveyor, () -> false),
                        new DelayedIntakeOn(intake, .25)),
                new ShootCargo(driveBase, shooter, conveyor,
                        limelight, true),
                new InstantCommand(() -> intake.setSpeed(0)),
                new SwerveControllerFollower(driveBase, fiveBallTrajectory3).deadlineWith(
                        new ConveyorDefault(conveyor, () -> false)),
                new InstantCommand(() -> intake.setSpeed(1)),
                new SwerveControllerFollower(driveBase, fiveBallTrajectory4).deadlineWith(
                        new ConveyorDefault(conveyor, () -> false)),
                new WaitCommand(1).deadlineWith(
                        new ConveyorDefault(conveyor, () -> false)),
                new SwerveControllerFollower(driveBase, fiveBallTrajectory5)
                        .deadlineWith(new ConveyorDefault(conveyor, () -> false)),
                new InstantCommand(() -> intake.setSpeed(0)),
                new InstantCommand(() -> intake.unDeploy()),
                new ShootCargo(driveBase, shooter, conveyor, limelight, true),
                new InstantCommand(() -> driveBase.resetOdometryWithLastHeading()));

        addRequirements(driveBase, intake, shooter, conveyor, limelight);

    }
}