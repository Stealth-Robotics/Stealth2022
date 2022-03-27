package frc.robot.commands.ConveyerCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

public class ConveyorDefault extends CommandBase {

    private final Conveyor conveyor;
    private final BooleanSupplier reverseIntake;

    public ConveyorDefault(Conveyor conveyor, BooleanSupplier reverseIntakeSpeed) {
        this.conveyor = conveyor;
        this.reverseIntake = reverseIntakeSpeed;

        addRequirements(conveyor);
    }

    @Override
    public void execute() {

        // if (reverseIntake.getAsBoolean()) {
        //     conveyor.setSpeed(-0.4);
        // } else 
        if (conveyor.getBreak() && conveyor.getSpeed() != 0.3) {
            conveyor.setSpeed(0.3);
        } else if (conveyor.getSpeed() != 0.0) {
            conveyor.setSpeed(0);
        }

        System.out.println("Conveyor Defualt");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}