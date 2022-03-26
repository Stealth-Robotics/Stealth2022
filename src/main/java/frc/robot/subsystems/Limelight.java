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

    private void intializeLimelight() {
        setLedMode(3);
        setCamMode(0);
    }

    /**
     * Wether the Limelight has a valid target
     * 
     * @return A boolean that is true when the Limelight has a valid target.
     */
    public boolean hasValidTarget() {
        return limelightTableEntry.getEntry("tv").getDouble(0) == 1;
    }

    /**
     * Gets horizontal offset of the Limelight's crosshair
     * 
     * @return The horizontal offset of the Limelight's crosshair.
     */
    public double getTargetHorizontalOffset() {
        return limelightTableEntry.getEntry("tx").getDouble(0.0);
    }

    /**
     * Gets vertical offset of the Limelight's crosshair
     * 
     * @return The vertical offset of the Limelight's crosshair.
     */
    public double getTargetVerticalOffset() {
        return limelightTableEntry.getEntry("ty").getDouble(0.0);
    }

    /**
     * Gets the targets area in relation the the image size
     * 
     * @return The area of the target in relationship to the image size.
     */
    public double getTargetArea() {
        return limelightTableEntry.getEntry("ta").getDouble(0.0);
    }

    /**
     * Gets distance from the front of the shooter to the target.
     * 
     * @return The distance from the shooter to the LimeLight target.
     */
    public double getTargetDistance() {

        if (!hasValidTarget())
            return 0.0;

        double angleToGoalDegrees = Constants.LimelightConstants.MOUNTED_ANGLE + getTargetVerticalOffset();
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

        return ((Constants.LimelightConstants.TARGET_HEIGHT - Constants.LimelightConstants.LENS_HEIGHT)
                / Math.tan(angleToGoalRadians));

    }

    /**
     * Gets the current camera mode of the Limelight.
     * 
     * @param defaultValue The default value if the NetworkTableEntry could not be
     *                     found.
     * @return The current camera mode of the Limelight.
     */
    public double getCamMode(double defaultValue) {
        return limelightTableEntry.getEntry("camMode").getDouble(defaultValue);
    }

    /**
     * Gets the current LED mode of the Limelight.
     * 
     * @param defaultValue The default value if the NetworkTableEntry could not be
     *                     found.
     * @return The current LED mode of the Limelight.
     */
    public double getLedMode(double defaultValue) {
        return limelightTableEntry.getEntry("ledMode").getDouble(defaultValue);
    }

    /**
     * Sets the camera mode of the Limelight to a given camera mode.
     * 
     * @param camMode The given camera mode to set the LimeLight to.
     */
    public void setCamMode(double camMode) {
        limelightTableEntry.getEntry("camMode").setNumber(camMode);
    }

    /**
     * Set the LED mode of the Limelight to a given LED mode.
     * 
     * @param ledMode The given LED mode to set the Limelight to.
     */
    public void setLedMode(double ledMode) {
        limelightTableEntry.getEntry("ledMode").setNumber(ledMode);
    }
}