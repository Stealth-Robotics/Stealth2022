package frc.robot.util;

import edu.wpi.first.wpilibj.Joystick;

public class LogitechGamepad extends Joystick {

    // TODO: Add Functions For The Rest Of Joystick Controls

    private final int leftStickXAxisID = 1;
    private final int leftStickYAxisID = 0;
    private final int rightStickXAxisID = 4;
    private final int rightStickYAxisID = 5;
    private final int leftTriggerAxisID = 2;
    private final int rightTriggerAxisID = 3;

    private final int aButtonID = 1;
    private final int bButtonID = 2;
    private final int xButtonID = 3;
    private final int yButtonID = 4;
    private final int leftBumperButtonID = 5;
    private final int rightBumperButtonID = 6;
    private final int backButtonID = 7;
    private final int startButtonID = 8;
    private final int leftJoystickButtonID = 9;
    private final int rightJoystickButtonID = 10;

    private double joystickDeadzone = 0.05;

    public LogitechGamepad(int port) {
        super(port);
    }

    public double getLeftStickX() {
        return getAxis(leftStickXAxisID);
    }

    public double getLeftStickY() {
        return -getAxis(leftStickYAxisID);
    }

    public double getRightStickX() {
        return getAxis(rightStickXAxisID);
    }

    public double getRightStickY() {
        return -getAxis(rightStickYAxisID);
    }

    public double getLeftTrigger() {
        return Math.abs(getAxis(leftTriggerAxisID));
    }

    public double getRightTrigger() {
        return Math.abs(getAxis(rightTriggerAxisID));
    }

    public boolean getAButton() {
        return getButton(aButtonID);
    }

    public boolean getBButton() {
        return getButton(bButtonID);
    }

    public boolean getXButton() {
        return getButton(xButtonID);
    }

    public boolean getYButton() {
        return getButton(yButtonID);
    }

    public boolean getLeftBumper() {
        return getButton(leftBumperButtonID);
    }

    public boolean getRightBumper() {
        return getButton(rightBumperButtonID);
    }

    public boolean getBackButton() {
        return getButton(backButtonID);
    }

    public boolean getStartButton() {
        return getButton(startButtonID);
    }

    public boolean getLeftJoystickButton() {
        return getButton(leftJoystickButtonID);
    }

    public boolean getRightJoystickButton() {
        return getButton(rightJoystickButtonID);
    }

    public double getAxis(int axisID) {
        double axisValue = getRawAxis(axisID);
        return Math.abs(axisValue) < joystickDeadzone ? 0.0 : axisValue;
    }

    public void setJoystickDeadZone(double newValue) {
        this.joystickDeadzone = newValue;
    }

    public boolean getButton(int buttonID) {
        return getRawButton(buttonID);
    }
}
