package frc.robot.commands.MultiSubsystemCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ConveyerCommands.MoveConveyor;
import frc.robot.commands.ShooterCommands.ReadyShooter;
import frc.robot.commands.ShooterCommands.ResetShooter;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;

public class ShootTopCargo extends SequentialCommandGroup {
    private final Conveyor conveyor;
    private final double distance;
    private final Shooter shooter;

    public ShootTopCargo(Shooter shooter, Conveyor conveyor, double distance) {
        this.conveyor = conveyor;
        this.distance  = distance;
        this.shooter = shooter;

        addCommands(
                new ReadyShooter(shooter, distance),
                new MoveConveyor(conveyor, Constants.Conveyor.SHOOT_CONVEYOR_STEP),
                new ResetShooter(shooter));

    }

    @Override
    public void end(boolean interrupted) {
        shooter.setVelocity(0);
        shooter.hoodToPos(0);
    }
}
