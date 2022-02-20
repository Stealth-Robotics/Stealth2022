package frc.robot.commands.ConveyerCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Conveyor;

public class ConveyorDefault extends CommandBase{

    private final Conveyor conveyor;
     
    public ConveyorDefault(Conveyor conveyor)
    {
        this.conveyor = conveyor;
    }

    @Override
    public void execute() {
        if(!conveyor.getBreak()) 
        {
            return;
        }

        conveyor.moveByPosition(Constants.Conveyor.CONVEYOR_STEP);
        //conveyor.addBall(conveyor.getBallColor);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
