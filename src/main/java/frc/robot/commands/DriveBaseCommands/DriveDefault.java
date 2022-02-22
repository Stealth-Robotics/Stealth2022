package frc.robot.commands.DriveBaseCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveDefault extends CommandBase {
    private final DriveBase driveBase;

    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier rotationSupplier;

    private final BooleanSupplier slowModeSupplier;

    private double xMetersPerSec = 0;
    private double yMetersPerSec = 0;
    private double omegaRadsPerSec = 0;

    private boolean slowMode = false;

    public DriveDefault(DriveBase driveBase,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier,
            BooleanSupplier slowModeSupplier) {
        this.driveBase = driveBase;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;
        this.slowModeSupplier = slowModeSupplier;

        addRequirements(driveBase);
    }

    @Override
    public void execute() {

        slowMode = slowModeSupplier.getAsBoolean();

        xMetersPerSec = slowMode
                ? (translationXSupplier.getAsDouble() * Constants.DriveBase.MAX_VELOCITY_METERS_PER_SECOND)
                        * Constants.DriveBase.SLOWMODE_MULTIPLIER
                : (translationXSupplier.getAsDouble() * Constants.DriveBase.MAX_VELOCITY_METERS_PER_SECOND);

        yMetersPerSec = slowMode
                ? (translationYSupplier.getAsDouble() * Constants.DriveBase.MAX_VELOCITY_METERS_PER_SECOND)
                        * Constants.DriveBase.SLOWMODE_MULTIPLIER
                : (translationYSupplier.getAsDouble() * Constants.DriveBase.MAX_VELOCITY_METERS_PER_SECOND);

        omegaRadsPerSec = slowMode
                ? (rotationSupplier.getAsDouble() * Constants.DriveBase.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)
                        * Constants.DriveBase.SLOWMODE_MULTIPLIER
                : (rotationSupplier.getAsDouble() * Constants.DriveBase.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);

        driveBase.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        xMetersPerSec,
                        yMetersPerSec,
                        omegaRadsPerSec,
                        driveBase.getGyroscopeRotation()));
    }

    @Override
    public void end(boolean interrupted) {
        driveBase.drive(0.0, 0.0, 0.0);
    }
}