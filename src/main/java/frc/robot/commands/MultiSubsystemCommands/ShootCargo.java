package frc.robot.commands.MultiSubsystemCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.ConveyerCommands.MoveConveyor;
import frc.robot.commands.ShooterCommands.FireShooter;

public class ShootCargo extends SequentialCommandGroup {

    public ShootCargo(DriveBase driveBase, Shooter shooter, Conveyor conveyor) {
        addRequirements(shooter, conveyor, driveBase);

        addCommands(
                // TODO: Align to Target Goes Here
                new FireShooter(shooter, 12),
                new WaitCommand(2),
                new MoveConveyor(conveyor)
        );
    }

}
