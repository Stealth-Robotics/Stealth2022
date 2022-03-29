package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class MoveClimber extends CommandBase {
    private final Climber climb;
    private final int newPos;
    private final double outputVeloMax;

    public MoveClimber(Climber c, int newpos, double maxVelo) {
        this.climb = c;
        this.newPos = newpos;
        this.outputVeloMax = maxVelo;
        addRequirements(climb);
    }

    @Override
    public void initialize() {
        climb.climberToPos(newPos);

    }

    @Override
    public void execute() {
        climb.updateVelo(outputVeloMax);
    }

    @Override
    public boolean isFinished() {
        return climb.climberAtPos();
    }
}
