package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
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
        //shooter.hoodToDegree(distanceToDegree(distance));
        shooter.hoodToPos(2048);
        shooter.setVelocity(exitVeloToRPM(distanceToExitVelo(distance)));

    }

    @Override
    public boolean isFinished() {
        return shooter.atVelocity() && shooter.hoodAtPos();
    }


    private double distanceToDegree(double distance) {
        return Math.max(Constants.Shooter.HOOD_UPPER_BOUND, Math.min(Constants.Shooter.HOOD_LOWER_BOUND,
                108.365 - 6.35879 * Math.log(109.988 * distance - 167.543)));
    }

    private double exitVeloToRPM(double exitVelo) {
        return exitVelo * 91.673;
    }

    private double distanceToExitVelo(double distance)
    {
        return Math.max(0, Math.min(38, (0.172641*Math.pow(distance, 1.45093)) + 22.9122));
    }


    @Override
    public void end(boolean interrupted) {
        System.out.println("End Ready Shooteer%");
    }
}
