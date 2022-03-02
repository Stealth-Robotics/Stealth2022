package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class ReadyShooter extends CommandBase {
    private final Shooter shooter;
    private final double distance;

    public ReadyShooter(Shooter shooter, double distance) {
        this.shooter = shooter;
        this.distance = distance;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.hoodToDegree(distance);
        shooter.setVelocity(distanceToRpm(distance));
    }

    @Override
    public boolean isFinished() {
        return shooter.hoodAtPos();
    }

    // 272.75\sin\left(13.9266-0.0152556x\right)-180.267
    private double distanceToDegree(double distance) {
        return Math.min(
                Constants.Shooter.HOOD_LOWER_BOUND,
                Math.max(
                        Constants.Shooter.HOOD_UPPER_BOUND,
                        (272.75 * Math.sin(13.9266 - 0.0152556 * distance) - 180.267)));
    }

    private double distanceToRpm(double distance) {
        return Math.max(
                0,
                (8.70162 * Math.pow(distance, 1.68446) + 2867.95));
    }
}
