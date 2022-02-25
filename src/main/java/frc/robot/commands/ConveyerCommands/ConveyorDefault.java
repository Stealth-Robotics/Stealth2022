package frc.robot.commands.ConveyerCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

public class ConveyorDefault extends CommandBase {

    private final Conveyor conveyor;

    public ConveyorDefault(Conveyor conveyor) {
        this.conveyor = conveyor;

        addRequirements(conveyor);
    }

    @Override
    public void execute() {
        if (conveyor.getBreak() && conveyor.getSpeed() != 0.4) {
            conveyor.setSpeed(0.4);
        }

        else if(conveyor.getSpeed() != 0.0) {
            conveyor.setSpeed(0);
        }
    }
}