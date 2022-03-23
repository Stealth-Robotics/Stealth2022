package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Climber;

public class AutoClimb extends SequentialCommandGroup{
    private final Climber climb;


    public AutoClimb(Climber c) {
        this.climb = c;
        addRequirements(climb);

        addCommands(
                         //climb to mid
        new InstantCommand(() -> climb.movePisitons(false)),
        new MoveClimber(climb, 0),

        //climb to high
        new InstantCommand(() -> climb.movePisitons(true)),
        new MoveClimber(climb, 20000),
        new WaitCommand(.5),
        new MoveClimber(climb, 25000),
        new InstantCommand(() -> climb.movePisitons(false)),
        new WaitCommand(.5), 
        new MoveClimber(climb, 0),
        
        //climb to traverse
        new InstantCommand(() -> climb.movePisitons(true)),
        new MoveClimber(climb, 20000),
        new WaitCommand(.5),
        new MoveClimber(climb, 25000),

        new InstantCommand(() -> climb.movePisitons(false)),
        new WaitCommand(.5),
        new MoveClimber(climb, 10000)
        );

    }

}
