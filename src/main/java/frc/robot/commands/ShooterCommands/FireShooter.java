package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Shooter;

public class FireShooter extends ParallelCommandGroup {

    public FireShooter(Shooter shooter, double distance) {
        addRequirements(shooter);

        addCommands(
                new AimHood(shooter, distance),
                new SpinShooter(shooter, distance));
    }

}
