package frc.robot.commands.DriveBaseCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Limelight;
import frc.robot.utils.HoodPIDController;

public class AlignWithTarget extends CommandBase {

    // TODO: PID Controller Needs to be Tuned
    private final DriveBase driveBase;
    private final Limelight limelight;
    private final HoodPIDController alignController;
    private double newpower;

    public AlignWithTarget(DriveBase driveBase, Limelight limelight) {
        this.driveBase = driveBase;
        this.limelight = limelight;
        alignController = new HoodPIDController(
                Constants.DriveBaseConstants.ALIGN_P_COEFF,
                Constants.DriveBaseConstants.ALIGN_I_COEFF,
                Constants.DriveBaseConstants.ALIGN_D_COEFF);

        alignController.setTolerance(Constants.DriveBaseConstants.ALIGN_TOLERANCE);

        addRequirements(driveBase, limelight);
    }

    @Override
    public void initialize() {
        alignController.setSetpoint(0);
    }

    public void execute() {
        newpower = alignController.calculate(limelight.getTargetHorizontalOffset());
        if (newpower > 5.75)
            newpower = 5.75;
        if (newpower < -5.75)
            newpower = -5.75;
        driveBase.drive(0, 0, newpower);
    }

    @Override
    public void end(boolean interrupted) {
        driveBase.drive(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return alignController.atSetpoint() || !limelight.hasValidTarget();
    }

}
