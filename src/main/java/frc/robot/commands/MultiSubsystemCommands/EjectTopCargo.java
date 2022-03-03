package frc.robot.commands.MultiSubsystemCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ConveyerCommands.MoveConveyor;
import frc.robot.commands.ShooterCommands.ResetShooter;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;

public class EjectTopCargo extends SequentialCommandGroup {
    private final Conveyor conveyor;
    private final Shooter shooter;

    public EjectTopCargo(Shooter shooter, Conveyor conveyor) {
        this.conveyor = conveyor;
        this.shooter = shooter;

        addRequirements(conveyor, shooter);

        addCommands(
                new ParallelCommandGroup(
                        new InstantCommand(() -> shooter.setVelocity(1000)),
                        new InstantCommand(() -> shooter.hoodToDegree(70))),
                new MoveConveyor(conveyor, Constants.ConveyorConstants.SHOOT_CONVEYOR_STEP),
                new ResetShooter(shooter));

    }

    @Override
    public void end(boolean interrupted) {
        shooter.setVelocity(0);
        shooter.hoodToPos(0);
    }
}
