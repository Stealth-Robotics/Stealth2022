package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class SpinShooter extends CommandBase {
    private final Shooter shooter;

    private final double distance;

    public SpinShooter(Shooter shooter, double distance) {
        this.shooter = shooter;
        this.distance = distance;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setVelocity(exitVeloToRPM(distanceToExitVelo(distance)));

    }

    private double exitVeloToRPM(double exitVelo) {
        return exitVelo * 91.673;
    }

    private double distanceToExitVelo(double distance)
    {
        return Math.max(0, Math.min(38, (0.172641*Math.pow(distance, 1.45093)) + 22.9122));
    }

}
