package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;

public class DelayedIntakeOn extends SequentialCommandGroup {
    public DelayedIntakeOn(Intake intake, double delay) {
        addCommands(
                new WaitCommand(delay),
                new InstantCommand(() -> intake.setSpeed(1)));
    }

}
