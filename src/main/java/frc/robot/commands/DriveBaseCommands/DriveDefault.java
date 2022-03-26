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

        driveBase.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        (Math.abs(translationXSupplier
                                .getAsDouble()) < Constants.IOConstants.DRIVE_JOYSTICK_DEADZONE)
                                        ? 0
                                        : (slowModeSupplier.getAsBoolean()
                                                ? translationXSupplier
                                                        .getAsDouble()
                                                        * Constants.DriveBaseConstants.MAX_VELOCITY_METERS_PER_SECOND
                                                        / 4
                                                : translationXSupplier
                                                        .getAsDouble()
                                                        * Constants.DriveBaseConstants.MAX_VELOCITY_METERS_PER_SECOND),
                        (Math.abs(translationYSupplier
                                .getAsDouble()) < Constants.IOConstants.DRIVE_JOYSTICK_DEADZONE)
                                        ? 0
                                        : (slowModeSupplier.getAsBoolean()
                                                ? translationYSupplier
                                                        .getAsDouble()
                                                        * Constants.DriveBaseConstants.MAX_VELOCITY_METERS_PER_SECOND
                                                        / 4
                                                : translationYSupplier
                                                        .getAsDouble()
                                                        * Constants.DriveBaseConstants.MAX_VELOCITY_METERS_PER_SECOND),
                        (Math.abs(rotationSupplier
                                .getAsDouble()) < Constants.IOConstants.DRIVE_JOYSTICK_DEADZONE)
                                        ? 0
                                        : (slowModeSupplier.getAsBoolean()
                                                ? rotationSupplier
                                                        .getAsDouble()
                                                        * Constants.DriveBaseConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
                                                        / 4
                                                : rotationSupplier
                                                        .getAsDouble()
                                                        * Constants.DriveBaseConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND),
                        driveBase.getGyroscopeRotation()));
    }

    @Override
    public void end(boolean interrupted) {
        driveBase.drive(0.0, 0.0, 0.0);
    }
}