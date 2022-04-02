package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class ReadyShooter extends CommandBase {
    private final Shooter shooter;
    private final Limelight limelight;

    public ReadyShooter(Shooter shooter, Limelight limelight) {
        this.shooter = shooter;
        this.limelight = limelight;
        addRequirements(shooter, limelight);
    }

    @Override
    public void initialize() {
        shooter.hoodToDegree(distanceToDegree(limelight.getTargetDistance() / 12.0));
        shooter.setVelocity(distanceToRpm(limelight.getTargetDistance() / 12.0));
       // shooter.hoodToDegree(58.5);
        //shooter.setVelocity(4900);

    }

    @Override
    public boolean isFinished() {
        return shooter.hoodAtPos();
    }

    private double distanceToDegree(double adistance) {
        return Math.min(Constants.ShooterConstants.HOOD_LOWER_BOUND,
               // Math.max(Constants.ShooterConstants.HOOD_UPPER_BOUND,
                     //   ((0.065053 * Math.pow(adistance, 2)) - (3.1419 * adistance) + 96.0155)));
                        Math.max(Constants.ShooterConstants.HOOD_UPPER_BOUND,
                        ((0.0458333 * Math.pow(adistance, 2)) - (2.71417 * adistance) + 94.0217)));

    }

    private double distanceToRpm(double distance) {
        return Math.max(
                0,
                (2.32207 * Math.pow(distance, 2) + (17.2618 * distance) + 2710.53));
    }
}
