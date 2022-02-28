package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Conveyor.BALL_COLORS;
import frc.robot.RobotMap;

public class Conveyor extends SubsystemBase {
    private final WPI_TalonFX conveyorMotor;
    private final DigitalInput beamBreak;
    private double target = 0;

    private BALL_COLORS topBallColor = BALL_COLORS.EMPTY;
    private BALL_COLORS bottomBallColor = BALL_COLORS.EMPTY;

    public Conveyor() {
        conveyorMotor = new WPI_TalonFX(RobotMap.Conveyor.CONVEYER_MOTOR);
        beamBreak = new DigitalInput(RobotMap.Conveyor.BEAM_BREAK);

        conveyorMotor.configFactoryDefault();
        conveyorMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                Constants.Conveyor.PID_LOOP_IDX, Constants.Conveyor.TIMEOUT);

        conveyorMotor.configNominalOutputForward(0, Constants.Conveyor.TIMEOUT);
        conveyorMotor.configNominalOutputReverse(0, Constants.Conveyor.TIMEOUT);
        conveyorMotor.configPeakOutputForward(0.4, Constants.Conveyor.TIMEOUT);
        conveyorMotor.configPeakOutputReverse(-0.4, Constants.Conveyor.TIMEOUT);

        conveyorMotor.setInverted(true);

    }

    public void moveByAmount(double amount) {
        target = amount + getConveyorPosition();
    }

    public void setSpeed(double speed) {
        conveyorMotor.set(ControlMode.PercentOutput, speed);
    }

    public double getSpeed() {
        return conveyorMotor.getMotorOutputPercent();
    }

    public double getConveyorPosition() {
        return conveyorMotor.getSelectedSensorPosition();
    }

    public void setConveyorEncoderPosition(double pos) {
        conveyorMotor.setSelectedSensorPosition(pos);
    }

    public boolean getBreak() {
        return !beamBreak.get();
    }

    public boolean atPosition() {
        return target >= 0 ? conveyorMotor.getSelectedSensorPosition() >= target
                : conveyorMotor.getSelectedSensorPosition() <= target;
    }

    public void addBall(BALL_COLORS newColor) {
        if (bottomBallColor == BALL_COLORS.EMPTY)
            bottomBallColor = newColor;

        else {
            topBallColor = bottomBallColor;
            bottomBallColor = newColor;
        }
    }

    public boolean hasBall() {
        return bottomBallColor != BALL_COLORS.EMPTY || topBallColor != BALL_COLORS.EMPTY;
    }

    public void removeTopBall() {
        topBallColor = bottomBallColor;
        bottomBallColor = BALL_COLORS.EMPTY;
    }

    public void removeBottomBall() {
        bottomBallColor = topBallColor;
        topBallColor = BALL_COLORS.EMPTY;
    }

    public BALL_COLORS getTopBall() {
        return topBallColor;
    }

    public BALL_COLORS getBottomBall() {
        return bottomBallColor;
    }

    public boolean isFull() {
        return !(bottomBallColor == BALL_COLORS.EMPTY && topBallColor == BALL_COLORS.EMPTY);
    }

    @Override
    public void periodic() {

        if (!atPosition() && getSpeed() != 0.4) {
            setSpeed(0.4);
        }

        else if (getSpeed() != 0.0) {
            setSpeed(0);
        }

    }
}