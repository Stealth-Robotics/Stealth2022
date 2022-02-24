package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {
    NetworkTable limelightTableEntry;

    public Limelight() {
        ShuffleboardTab tab = Shuffleboard.getTab("Limelight");

        limelightTableEntry = NetworkTableInstance.getDefault().getTable("limelight");

        intializeLimelight();

        tab.getLayout("Valid Target", BuiltInLayouts.kList)
                .withPosition(0, 0)
                .withSize(2, 1)
                .addBoolean("Valid Target Value", () -> hasValidTarget());

        tab.getLayout("Offset Values", BuiltInLayouts.kList)
                .withPosition(2, 0)
                .withSize(2, 3)
                .addNumber("Horizontal Offset", () -> getTargetHorizontalOffset());

        tab.getLayout("Offset Values", BuiltInLayouts.kList)
                .withPosition(2, 0)
                .withSize(2, 3)
                .addNumber("Vertical Offset", () -> getTargetVerticalOffset());

        tab.getLayout("Target Distance", BuiltInLayouts.kList)
                .withPosition(0, 1)
                .withSize(2, 2)
                .addNumber("Distance", () -> getTargetDistance());
    }

    public void intializeLimelight() {
        setLedMode(3);
        setCamMode(0);
    }

    public boolean hasValidTarget() {
        return limelightTableEntry.getEntry("tv").getDouble(0) == 1;
    }

    public double getTargetHorizontalOffset() {
        return limelightTableEntry.getEntry("tx").getDouble(0.0);
    }

    public double getTargetVerticalOffset() {
        return limelightTableEntry.getEntry("ty").getDouble(0.0);
    }

    public double getTargetArea() {
        return limelightTableEntry.getEntry("ta").getDouble(0.0);
    }

    public double getTargetDistance() {

        if(!hasValidTarget())
            return 0.0;

        double angleToGoalDegrees = Constants.Limelight.MOUNTED_ANGLE + getTargetVerticalOffset();
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        return ((Constants.Limelight.TARGET_HEIGHT - Constants.Limelight.LENS_HEIGHT) / Math.tan(angleToGoalRadians)) + Constants.Limelight.LENS_TO_SHOOTER;
    }

    public void getCamMode(double defaultValue) {
        limelightTableEntry.getEntry("camMode").getDouble(defaultValue);
    }

    public void getLedMode(double defaultValue) {
        limelightTableEntry.getEntry("ledMode").getDouble(defaultValue);
    }

    public void setCamMode(double camMode) {
        limelightTableEntry.getEntry("camMode").setNumber(camMode);
    }

    public void setLedMode(double ledMode) {
        limelightTableEntry.getEntry("ledMode").setNumber(ledMode);
    }
}