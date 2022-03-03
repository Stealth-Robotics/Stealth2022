package frc.robot.commands.MultiSubsystemCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ConveyerCommands.MoveConveyor;
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

    public ShootCargo(DriveBase driveBase, Shooter shooter, Conveyor conveyor,Limelight limelight ) {

        this.driveBase = driveBase;
        this.shooter = shooter;
        this.conveyor = conveyor;
        this.limelight = limelight;

        addRequirements(shooter, conveyor, driveBase);

        addCommands(
                new MoveConveyor(conveyor, -5000),
                new ReadyShooter(shooter, (limelight.getTargetDistance()/12)),
                new MoveConveyor(conveyor, Constants.ConveyorConstants.SHOOT_CONVEYOR_STEP * 2),
                new ResetShooter(shooter));
    }

    @Override
    public void end(boolean interrupted) {
        driveBase.drive(0, 0, 0);
        shooter.setVelocity(0);
        shooter.hoodToPos(0);
    }
}
