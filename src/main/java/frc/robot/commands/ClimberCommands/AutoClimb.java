package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Climber;

public class AutoClimb extends SequentialCommandGroup {
    private final Climber climber;

    public AutoClimb(Climber c) {
        this.climber = c;
        addRequirements(climber);

        addCommands(
                // climb to mid
                new InstantCommand(() -> climber.movePisitons(false)),
                new MoveClimber(climber, 0),

               // climb to high
                new InstantCommand(() -> climber.movePisitons(true))
               /* new MoveClimber(climber, 20000),
                new WaitCommand(.5),
                new MoveClimber(climber, 25000),
                new InstantCommand(() -> climber.movePisitons(false)),
                new WaitCommand(.5),
                new MoveClimber(climber, 0),

                // climb to traverse
                new InstantCommand(() -> climber.movePisitons(true)),
                new MoveClimber(climber, 20000),
                new WaitCommand(.5),
                new MoveClimber(climber, 25000),

                new InstantCommand(() -> climber.movePisitons(false)),
                new WaitCommand(.5),
                new MoveClimber(climber, 10000)
                */
                );
                

    }

}
