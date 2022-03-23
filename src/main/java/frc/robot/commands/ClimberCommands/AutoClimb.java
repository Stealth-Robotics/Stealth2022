package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Climber;

public class AutoClimb extends SequentialCommandGroup{
    private final Climber climb;


    public AutoClimb(Climber c) {
        this.climb = c;
        addRequirements(climb);
    }


    @Override
    public void initialize() {

        //climb to mid
        climb.movePisitons(false);
        climb.climberToPos(0);
        //TODO wait until finished

        //climb to high
        climb.movePisitons(true);
        climb.climberToPos(20000);
        //TODO wait until finished
        climb.climberToPos(25000);
        climb.movePisitons(false);
        new WaitCommand(.5); 
        climb.climberToPos(0);
        //TODO wait until finished
        
        //climb to traverse
        climb.movePisitons(true);
        climb.climberToPos(20000);
        new WaitCommand(.5);
        climb.climberToPos(25000);
        //TODO wait until finished
        climb.movePisitons(false);
        new WaitCommand(.5); 
        climb.climberToPos(10000);

    }

    public boolean isFinished() {
        return false; //TODO add end condition
    }

}
