package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DoNothingAuto extends SequentialCommandGroup{
    public DoNothingAuto(){
        addCommands(new InstantCommand(() -> System.out.println("Nothing auto")));
    }
}
