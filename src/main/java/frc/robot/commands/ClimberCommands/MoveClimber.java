package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class MoveClimber extends CommandBase{
    private final Climber climb;
    private final int newPos;


    public MoveClimber(Climber c, int newpos) {
        this.climb = c;
        this.newPos = newpos;
        addRequirements(climb);
    }

    @Override
    public void initialize() {
        climb.climberToPos(newPos);
    }

    @Override
    public boolean isFinished() {
        return climb.climberAtPos();
    }
}
