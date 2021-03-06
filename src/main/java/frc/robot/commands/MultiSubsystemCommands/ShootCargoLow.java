package frc.robot.commands.MultiSubsystemCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.ConveyerCommands.MoveConveyor;
import frc.robot.commands.DriveBaseCommands.AlignWithTarget;
import frc.robot.commands.ShooterCommands.ResetShooter;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Limelight;

public class ShootCargoLow extends SequentialCommandGroup {

    private final DriveBase driveBase;
    private final Shooter shooter;
    private final Conveyor conveyor;
    private final Limelight limelight;
    // private double distance;

    public ShootCargoLow(DriveBase driveBase, Shooter shooter, Conveyor conveyor, Limelight limelight) {

        this.driveBase = driveBase;
        this.shooter = shooter;
        this.conveyor = conveyor;
        this.limelight = limelight;

        addRequirements(shooter, conveyor, driveBase, limelight);

        addCommands(
                new MoveConveyor(conveyor, -500),
                new InstantCommand(() -> shooter.setSpeed(.2)),
                new InstantCommand(() -> shooter.hoodToDegree(60)),
                new WaitUntilCommand(() -> shooter.hoodAtPos()),
                new MoveConveyor(conveyor, Constants.ConveyorConstants.SHOOT_CONVEYOR_STEP * 2),
                new ResetShooter(shooter));
    }

    @Override
    public void end(boolean interrupted) {
        driveBase.drive(0, 0, 0);
        shooter.setSpeed(0);
        shooter.hoodToPos(0);
        super.end(false);
    }
}
