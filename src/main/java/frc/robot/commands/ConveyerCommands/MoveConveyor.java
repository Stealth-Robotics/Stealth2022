package frc.robot.commands.ConveyerCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

public class MoveConveyor extends CommandBase {
    private final Conveyor conveyor;

    private final double distance;
    private final double speed;

    private double startingDistance;

    public MoveConveyor(Conveyor conveyor, double distance, double speed) {
        this.conveyor = conveyor;
        this.distance = distance;
        this.speed = speed;

        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
        startingDistance = conveyor.getConveyorPosition();
        conveyor.setSpeed(speed);
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