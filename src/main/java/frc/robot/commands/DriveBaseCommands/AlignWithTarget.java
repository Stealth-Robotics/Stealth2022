package frc.robot.commands.DriveBaseCommands;

import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.Constants;
import frc.robot.subsystems.DriveBase;

public class AlignWithTarget extends CommandBase {

    // TODO: PID Controller Needs to be Tuned
    private final DriveBase driveBase;
    private final Limelight limelight;
    private final PIDController alignController;

    public AlignWithTarget(DriveBase driveBase, Limelight limelight) {
        this.driveBase = driveBase;
        this.limelight = limelight;
        alignController = new PIDController(
                Constants.DriveBase.ALIGN_P_COEFF,
                Constants.DriveBase.ALIGN_I_COEFF,
                Constants.DriveBase.ALIGN_D_COEFF);

        alignController.setTolerance(Constants.DriveBase.ALIGN_TOLERANCE);
    }

    @Override
    public void initialize() {
        alignController.setSetpoint(0);
    }

    public void execute() {
        driveBase.drive(0, 0, alignController.calculate(limelight.getTargetHorizontalOffset()));
    }

    @Override
    public boolean isFinished() {
        return alignController.atSetpoint() || !limelight.hasValidTarget();
    }

}
