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
import frc.robot.commands.ConveyerCommands.MoveConveyor;
import frc.robot.commands.DriveBaseCommands.SwerveControllerFollower;
import frc.robot.commands.MultiSubsystemCommands.ShootCargo;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class TwoMinusOneBallAuto extends SequentialCommandGroup {

    final static PathPlannerTrajectory twoMinusOneBallTrajectory1 = PathPlanner.loadPath("2m1path1",
            0.5 * Constants.DriveBaseConstants.MAX_VELOCITY_METERS_PER_SECOND,
            1.0 * Constants.DriveBaseConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED, false);

    final static PathPlannerTrajectory twoMinusOneBallTrajectoryfix = PathPlanner.loadPath("2m1fix",
            0.5 * Constants.DriveBaseConstants.MAX_VELOCITY_METERS_PER_SECOND,
            1.0 * Constants.DriveBaseConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED, false);

    final static PathPlannerTrajectory twoMinusOneBallTrajectory2 = PathPlanner.loadPath("2m1path2",
            0.5 * Constants.DriveBaseConstants.MAX_VELOCITY_METERS_PER_SECOND,
            1.0 * Constants.DriveBaseConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED, false);

    final static PathPlannerTrajectory twoMinusOneBallTrajectory3 = PathPlanner.loadPath("2m1path3",
            0.5 * Constants.DriveBaseConstants.MAX_VELOCITY_METERS_PER_SECOND,
            1.0 * Constants.DriveBaseConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED, false);

    final static Pose2d initial = new Pose2d(
            twoMinusOneBallTrajectory1.getInitialPose().getTranslation(),
            ((PathPlannerState) twoMinusOneBallTrajectory1.sample(0)).holonomicRotation);

    public TwoMinusOneBallAuto(DriveBase driveBase, Intake intake, Shooter shooter, Conveyor conveyor,
            Limelight limelight) {

        addCommands(
                new InstantCommand(() -> intake.deploy()),
                new InstantCommand(() -> intake.setSpeed(1)),
                new SwerveControllerFollower(driveBase, twoMinusOneBallTrajectory1)
                        .beforeStarting(new InstantCommand(() -> driveBase
                                .resetOdometry(initial)))
                        .deadlineWith(new ConveyorDefault(conveyor, () -> false)),
                new WaitCommand(1).deadlineWith(new ConveyorDefault(conveyor, () -> false)),
                new InstantCommand(() -> intake.setSpeed(0)),
                new ShootCargo(driveBase, shooter, conveyor, limelight, true),
                new InstantCommand(() -> intake.setSpeed(1)),
                new SwerveControllerFollower(driveBase, twoMinusOneBallTrajectoryfix)
                        .deadlineWith(new ConveyorDefault(conveyor, () -> false)),
                new SwerveControllerFollower(driveBase, twoMinusOneBallTrajectory2)
                        .deadlineWith(new ConveyorDefault(conveyor, () -> false)),
                new InstantCommand(() -> intake.setSpeed(-0.3)),
                new MoveConveyor(conveyor, -20000),
                new WaitCommand(2),
                new InstantCommand(() -> conveyor.setSpeed(0)),
                new InstantCommand(() -> intake.setSpeed(0)),
                new InstantCommand(() -> intake.unDeploy()),
                new SwerveControllerFollower(driveBase, twoMinusOneBallTrajectory3),
                new InstantCommand(() -> driveBase.resetOdometryWithLastHeading()));

        addRequirements(driveBase, intake, shooter, conveyor, limelight);

    }
}
