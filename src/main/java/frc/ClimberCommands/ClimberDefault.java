package frc.ClimberCommands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.json.simple.parser.Yytoken;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class ClimberDefault extends CommandBase {
    private final Climber climber;
    private final DoubleSupplier upSpeed;
    private final DoubleSupplier downSpeed;

    public ClimberDefault(Climber climber, DoubleSupplier upSpeed, DoubleSupplier downSpeed) {
        this.climber = climber;
        this.upSpeed = upSpeed;
        this.downSpeed = downSpeed;
    }

    @Override
    public void execute() {

        if (downSpeed.getAsDouble() > 0.05
                && climber.getClimberPosition() > Constants.Climber.MIN_THRESHOLD) {
            climber.setSpeed(-downSpeed.getAsDouble() * Constants.Climber.MAX_SPEED);
        }

        else if (climber.getClimberPosition() < Constants.Climber.MAX_THRESHOLD) {
            climber.setSpeed(upSpeed.getAsDouble() * Constants.Climber.MAX_SPEED);
        }

        if (downSpeed.getAsDouble() > 0.05 && climber.getClimberPosition() <= Constants.Climber.MAX_THRESHOLD / 2) {
            climber.movePisitons(false);
        } else if (upSpeed.getAsDouble() > 0.05 && climber.getClimberPosition() > Constants.Climber.MAX_THRESHOLD / 2) {
            climber.movePisitons(true);
        }

    }
}
