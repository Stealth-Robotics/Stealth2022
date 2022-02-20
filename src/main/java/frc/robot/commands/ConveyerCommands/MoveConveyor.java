package frc.robot.commands.ConveyerCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

public class MoveConveyor extends CommandBase {
    private final Conveyor conveyor;

    private final double distance;

    public MoveConveyor(Conveyor conveyor, double distance) {
        this.conveyor = conveyor;
        this.distance = distance;

        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
        conveyor.moveByPosition(distance);
    }

    @Override
    public boolean isFinished() {
        return conveyor.atPosition();
    }

}
