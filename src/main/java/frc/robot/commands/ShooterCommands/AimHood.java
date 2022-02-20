package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class AimHood extends CommandBase {
    private final Shooter shooter;
    private final double distance;

    public AimHood(Shooter shooter, double distance) {
        this.shooter = shooter;
        this.distance = distance;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.hoodToDegree(distanceToDegree(distance));
    }

    @Override
    public boolean isFinished() {
        return shooter.hoodAtPos();
    }

    private double distanceToDegree(double distance) {
        return Math.max(Constants.Shooter.HOOD_UPPER_BOUND, Math.min(Constants.Shooter.HOOD_LOWER_BOUND,
                108.365 - 6.35879 * Math.log(109.988 * distance - 167.543)));
    }

}
