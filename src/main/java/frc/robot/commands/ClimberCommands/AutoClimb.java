package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Climber;

public class AutoClimb extends SequentialCommandGroup {
    private final Climber climber;

    public AutoClimb(Climber climber) {
        this.climber = climber;
        addRequirements(climber);

        addCommands(
                // climb to mid
               
               // climb to high
                new InstantCommand(() -> climber.movePisitons(true)),
                new WaitCommand(.0625),
                //new MoveClimber(climber, 65000),
                //new WaitCommand(.25),
                new MoveClimber(climber, 100000, 0.7)
                //new WaitCommand(.75),
               // new InstantCommand(() -> climber.movePisitons(false))
                /*new WaitCommand(.5),
                new MoveClimber(climber, 0),

                // climb to traverse
                new InstantCommand(() -> climber.movePisitons(true)),
                new WaitCommand(.25),
                new MoveClimber(climber, 65000),
                new WaitCommand(.25),
                new MoveClimber(climber, 97000),
                new InstantCommand(() -> climber.movePisitons(false)),
                new WaitCommand(.5),
                new MoveClimber(climber, 65000)               
                */ 
                );
                

    }

}
