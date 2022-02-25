package frc.robot.commands.ConveyerCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

public class MoveConveyor extends CommandBase {
    private final Conveyor conveyor;

    private final double distance;

    private double startingDistance;

    public MoveConveyor(Conveyor conveyor, double distance) {
        this.conveyor = conveyor;
        this.distance = distance;

        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
        startingDistance = conveyor.getConveyorPosition();
        conveyor.setSpeed(1);
    }

    @Override
    public boolean isFinished() {
        return conveyor.atPosition(startingDistance + distance);
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.setSpeed(0);
    }

}
