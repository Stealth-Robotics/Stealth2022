package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ResetShooter extends CommandBase {

    private final Shooter shooter;

    public ResetShooter(Shooter shooter) {
        this.shooter = shooter;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.hoodToPos(0);
        shooter.setVelocity(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
