package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.Climber;

public class RaiseClimber extends SequentialCommandGroup {
    private final Climber climb;


    public RaiseClimber(Climber c) {
        this.climb = c;
        addRequirements(climb);
    }

    @Override
    public void initialize() {
        climb.climberToPos(20000);
    }

    public boolean isFinished() {
        return climb.climberAtPos();
    }

}
