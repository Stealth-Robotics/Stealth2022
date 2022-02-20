package frc.robot.commands.ConveyerCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Conveyor;

public class MoveConveyor extends CommandBase {
    private final Conveyor conveyor;

    public MoveConveyor(Conveyor conveyor) {
        this.conveyor = conveyor;

        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
        conveyor.moveByPosition(Constants.Conveyor.CONVEYOR_STEP);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
