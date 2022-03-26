package frc.robot.commands.MultiSubsystemCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ConveyerCommands.MoveConveyor;
import frc.robot.commands.ShooterCommands.ReadyShooter;
import frc.robot.commands.ShooterCommands.ResetShooter;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class ShootTopCargo extends SequentialCommandGroup {
    private final Conveyor conveyor;
    private final Shooter shooter;
    private final Limelight limelight;

    public ShootTopCargo(Shooter shooter, Conveyor conveyor, Limelight limelight) {
        this.conveyor = conveyor;
        this.shooter = shooter;
        this.limelight = limelight;

        addRequirements(conveyor, shooter, limelight);

        addCommands(
                new ReadyShooter(shooter, limelight),
                new MoveConveyor(conveyor, Constants.ConveyorConstants.SHOOT_CONVEYOR_STEP),
                new ResetShooter(shooter));
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setVelocity(0);
        shooter.hoodToPos(0);
    }
}
