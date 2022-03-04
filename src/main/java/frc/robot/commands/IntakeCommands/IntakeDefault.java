package frc.robot.commands.IntakeCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeDefault extends CommandBase {
    private final Intake intake;

    private final DoubleSupplier forwardSpeed;
    private final DoubleSupplier reverseSpeed;

    public IntakeDefault(Intake intake, DoubleSupplier forwardSpeed, DoubleSupplier reverseSpeed) {
        this.intake = intake;
        this.forwardSpeed = forwardSpeed;
        this.reverseSpeed = reverseSpeed;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        if (reverseSpeed.getAsDouble() > 0.05) {
            intake.deploy();
            intake.setSpeed(-reverseSpeed.getAsDouble());
        } else if (forwardSpeed.getAsDouble() > 0.05) {
            intake.deploy();
            intake.setSpeed(forwardSpeed.getAsDouble());
        } else {
            intake.unDeploy();
            intake.setSpeed(0);
        }
    }

}
