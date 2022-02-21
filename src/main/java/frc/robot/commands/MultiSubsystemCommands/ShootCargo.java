package frc.robot.commands.MultiSubsystemCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.Conveyor.BALL_COLORS;
import frc.robot.commands.ConveyerCommands.MoveConveyor;
import frc.robot.commands.ShooterCommands.ReadyShooter;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Shooter;

public class ShootCargo extends SequentialCommandGroup {

    private final DriveBase driveBase;
    private final Shooter shooter;
    private final Conveyor conveyor;

    public ShootCargo(DriveBase driveBase, Shooter shooter, Conveyor conveyor) {

        this.driveBase = driveBase;
        this.shooter = shooter;
        this.conveyor = conveyor;

        addRequirements(shooter, conveyor, driveBase);

        addCommands(
                // Align With Target
                //new RunCommand(() -> driveBase.lockDriveBase(), driveBase),
                new ReadyShooter(shooter, 12));

        if (conveyor.getTopBall() == BALL_COLORS.UNKNOWN) {
            if (conveyor.getBottomBall() == BALL_COLORS.UNKNOWN) {
                addCommands(new MoveConveyor(conveyor, Constants.Conveyor.SHOOT_CONVEYOR_STEP * 2));
                conveyor.removeTopBall();
                conveyor.removeTopBall();
            }

            if (conveyor.getBottomBall() == BALL_COLORS.EMPTY) {
                addCommands(new MoveConveyor(conveyor, Constants.Conveyor.SHOOT_CONVEYOR_STEP));
                conveyor.removeTopBall();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        driveBase.drive(0, 0, 0);
        shooter.setSpeed(0);
    }
}
