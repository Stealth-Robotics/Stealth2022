package frc.robot.commands.ClimberCommands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class ClimberDefault extends CommandBase {
    private final Climber climber;
    private final DoubleSupplier upSpeed;
    private final DoubleSupplier downSpeed;
    private final BooleanSupplier override;
    private final BooleanSupplier overrideUp;
    private final BooleanSupplier toggleButton;

    public ClimberDefault(Climber climber, DoubleSupplier upSpeed, DoubleSupplier downSpeed, BooleanSupplier override,
            BooleanSupplier overrideUp, BooleanSupplier toggleButton) {
        this.climber = climber;
        this.upSpeed = upSpeed;
        this.downSpeed = downSpeed;
        this.override = override;
        this.overrideUp = overrideUp;
        this.toggleButton = toggleButton;

        addRequirements(climber);
    }

    @Override
    public void initialize() {
        climber.resetClimberEncoder();
    }

    @Override
    public void execute() {

        if (downSpeed.getAsDouble() > 0.05
                && climber.getClimberPosition() > Constants.ClimberConstants.MIN_THRESHOLD
                || override.getAsBoolean()) {
            climber.setSpeed(-downSpeed.getAsDouble() * Constants.ClimberConstants.MAX_SPEED);
        }

        else if (overrideUp.getAsBoolean()
                && climber.getClimberPosition() > Constants.ClimberConstants.MIN_THRESHOLD
                || override.getAsBoolean()) {
            climber.setSpeed(-Constants.ClimberConstants.MAX_SPEED);
        }

        else if (climber.getClimberPosition() < Constants.ClimberConstants.MAX_THRESHOLD
                && upSpeed.getAsDouble() > 0.05 || override.getAsBoolean()) {
            climber.setSpeed(upSpeed.getAsDouble() * Constants.ClimberConstants.MAX_SPEED);
        }

        else {
            climber.setSpeed(0);
        }

        if (downSpeed.getAsDouble() > 0.05
                && !override.getAsBoolean()) {
            climber.movePisitons(false);
        } else if (upSpeed.getAsDouble() > 0.05
                && climber.getClimberPosition() > Constants.ClimberConstants.MAX_THRESHOLD / 4
                && !override.getAsBoolean()) {
            climber.movePisitons(true);
        } else if (override.getAsBoolean() && toggleButton.getAsBoolean()) {
            climber.togglePivotPistons();
        }

    }
}
