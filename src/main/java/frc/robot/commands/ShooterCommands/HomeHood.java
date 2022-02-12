package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class HomeHood extends CommandBase {

    private final Shooter shooter;

    public HomeHood(Shooter shooter) {
        this.shooter = shooter;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setHoodSpeed(-0.3);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setHoodSpeed(0);
        shooter.setHoodEncoderPos(0);
    }

    @Override
    public boolean isFinished() {
        return shooter.getHoodSwitchState();
    }

}
