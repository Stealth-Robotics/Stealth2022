package frc.robot.commands.MultiSubsystemCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ConveyerCommands.MoveConveyor;
import frc.robot.commands.DriveBaseCommands.AlignWithTarget;
import frc.robot.commands.ShooterCommands.ReadyShooter;
import frc.robot.commands.ShooterCommands.ResetShooter;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Limelight;

public class ShootCargo extends SequentialCommandGroup {

    private final DriveBase driveBase;
    private final Shooter shooter;
    private final Conveyor conveyor;
    private final Limelight limelight;
    // private double distance;

    public ShootCargo(DriveBase driveBase, Shooter shooter, Conveyor conveyor, Limelight limelight) {

        this.driveBase = driveBase;
        this.shooter = shooter;
        this.conveyor = conveyor;
        this.limelight = limelight;

        // distance = this.limelight.getTargetDistance();

        addRequirements(shooter, conveyor, driveBase, limelight);

        if (limelight.hasValidTarget()) {
            addCommands(
                    // new InstantCommand(() -> System.out.println(limelight.getTargetDistance())),
                    new ParallelCommandGroup(
                            new AlignWithTarget(driveBase, this.limelight),
                            new MoveConveyor(conveyor, -500)),
                    new ReadyShooter(shooter, this.limelight),
                    new MoveConveyor(conveyor, Constants.ConveyorConstants.SHOOT_CONVEYOR_STEP * 2),
                    new ResetShooter(shooter));
        }

        else {
            addCommands(new InstantCommand());
        }
    }

    // @Override
    // public void execute() {
    // System.out.println(limelight.getTargetDistance());
    // }

    @Override
    public void end(boolean interrupted) {
        driveBase.drive(0, 0, 0);
        shooter.setSpeed(0);
        shooter.hoodToPos(0);
    }
}
