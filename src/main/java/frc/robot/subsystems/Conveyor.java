package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Conveyor.BALL_COLORS;
import frc.robot.RobotMap;

public class Conveyor extends SubsystemBase {
    private final WPI_TalonFX conveyorMotor;
    private final DigitalInput beamBreak;

    private final PIDController conveyorController;

    private BALL_COLORS topBallColor = BALL_COLORS.UNKNOWN;
    private BALL_COLORS bottomBallColor = BALL_COLORS.UNKNOWN;

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

        conveyorMotor.configAllowableClosedloopError(Constants.Conveyor.PID_LOOP_IDX,
                Constants.Conveyor.TOLERANCE, Constants.Conveyor.TIMEOUT);

        conveyorMotor.setInverted(true);

        conveyorController = new PIDController(
                Constants.Conveyor.CONVEOR_P_COEFF,
                Constants.Conveyor.CONVEYOR_I_COEFF,
                Constants.Conveyor.CONVEYOR_D_COEFF);

        conveyorController.setTolerance(Constants.Conveyor.TOLERANCE);
    }

    public void setSpeed(double speed) {
        conveyorMotor.set(ControlMode.PercentOutput, speed);
    }

    public void setPos(double newPos) {
        conveyorController.setSetpoint(newPos);
    }

    public void moveByPos(double newPos) {
        setPos(newPos + getConveyorPosition());
    }

    public double getConveyorPosition() {
        return conveyorMotor.getSelectedSensorPosition();
    }

    public void setConveyorEncoderPosition(double pos) {
        conveyorMotor.setSelectedSensorPosition(pos);
    }

    public boolean getBreak() {
        return beamBreak.get();
    }

    public boolean atPosition() {
        return conveyorController.atSetpoint();
    }

    @Override
    public void periodic() {
        setSpeed(conveyorController.calculate(getConveyorPosition()));
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
}
