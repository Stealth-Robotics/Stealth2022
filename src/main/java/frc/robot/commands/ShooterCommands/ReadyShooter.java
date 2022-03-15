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
        //shooter.hoodToDegree(70);
        // shooter.setVelocity(4400);

    }

    @Override
    public boolean isFinished() {
        return shooter.hoodAtPos();
    }

    // 272.75\sin\left(13.9266-0.0152556x\right)-180.267
    private double distanceToDegree(double adistance) {
        return Math.min(Constants.ShooterConstants.HOOD_LOWER_BOUND,
                Math.max(Constants.ShooterConstants.HOOD_UPPER_BOUND,
                        ((-0.00520833 * Math.pow(adistance, 2)) - (1.83036 * adistance) + 91.1674)));

        // return Math.min(
        // Constants.ShooterConstants.HOOD_LOWER_BOUND,
        // Math.max(
        // Constants.ShooterConstants.HOOD_UPPER_BOUND,
        // (272.75 * Math.sin(13.9266 - 0.0152556 * distance) - 180.267)));
    }

    private double distanceToRpm(double distance) {
        return Math.max(
                0,
                (6.77083 * Math.pow(distance, 2) - (74.1071 * distance) + 3265.85));
    }
}
