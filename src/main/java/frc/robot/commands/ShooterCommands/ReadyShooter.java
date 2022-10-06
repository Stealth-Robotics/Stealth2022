package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class ReadyShooter extends CommandBase {
    private final Shooter shooter;
    private final Limelight limelight;
    private final double overrideDistance;
    private final double adjustmentDist;

    public ReadyShooter(Shooter shooter, Limelight limelight, double distance, double adjustment) {
        this.shooter = shooter;
        this.limelight = limelight;
        this.overrideDistance = distance; // override limelight distance in auto
        this.adjustmentDist = adjustment;    
        addRequirements(shooter, limelight);
    }

    @Override
    public void initialize() {
        // if (overrideDistance == 0) {
             shooter.hoodToDegree(distanceToDegree(limelight.getTargetDistance() / 12.0));
             shooter.setVelocity(distanceToRpm(limelight.getTargetDistance() / 12.0));
        // } else {
        //     shooter.hoodToDegree(distanceToDegree(overrideDistance / 12.0));
        //     shooter.setVelocity(distanceToRpm(overrideDistance / 12.0));
        // }
        // shooter.hoodToDegree(69);
        //  shooter.setVelocity(4000);

    }

    @Override
    public boolean isFinished() {
        return shooter.hoodAtPos();
    }

    private double distanceToDegree(double adistance) {
        return Math.min(Constants.ShooterConstants.HOOD_LOWER_BOUND,
                Math.max(Constants.ShooterConstants.HOOD_UPPER_BOUND,
                        ((0.065053 * Math.pow(adistance, 2)) - (3.1419 * adistance) + 96.0155) + 25));

    }

    private double distanceToRpm(double distance) {
        return Math.max(
                0,
                (2.32207 * Math.pow(distance, 2) + (17.2618 * distance) + 2710.53 + 15));
    }
}
