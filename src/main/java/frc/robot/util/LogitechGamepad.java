package frc.robot.util;

import edu.wpi.first.wpilibj.Joystick;

public class LogitechGamepad extends Joystick {

    // TODO: Add Functions For The Rest Of Joystick Controls

    private final int leftStickXAxisID = 1;
    private final int leftStickYAxisID = 0;
    private final int rightStickXAxisID = 4;
    private final int rightStickYAxisID = 3; // TODO: Fix Right Stick Y Axis ID Based On Logitech Gamepad

    private final int aButtonID = 1;

    private double joystickDeadzone = 0.05;

    public LogitechGamepad(int port) {
        super(port);
    }

    // TODO: Check If Axes Are In The Correct Direction

    public double getLeftStickX() {
        return getAxis(leftStickXAxisID);
    }

    public double getLeftStickY() {
        return getAxis(leftStickYAxisID);
    }

    public double getRightStickX() {
        return getAxis(rightStickXAxisID);
    }

    public double getRightStickY() {
        return getAxis(rightStickYAxisID);
    }

    public boolean getAButton() {
        return getButton(aButtonID);
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
