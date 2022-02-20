package frc.robot.subsystems;

import java.util.Queue;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Conveyor.BALL_COLORS;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Conveyor extends SubsystemBase {
    private final WPI_TalonFX conveyorMotor;
    private final DigitalInput beamBreak;

    private Queue<BALL_COLORS> currentBalls;

    public Conveyor() {
        conveyorMotor = new WPI_TalonFX(RobotMap.Conveyor.CONVEYER_MOTOR);
        beamBreak = new DigitalInput(RobotMap.Conveyor.BEAM_BREAK);

        conveyorMotor.configFactoryDefault();
        conveyorMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                Constants.Conveyor.PID_LOOP_IDX, Constants.Conveyor.TIMEOUT);

        conveyorMotor.configNominalOutputForward(0, Constants.Conveyor.TIMEOUT);
        conveyorMotor.configNominalOutputReverse(0, Constants.Conveyor.TIMEOUT);
        conveyorMotor.configPeakOutputForward(1, Constants.Conveyor.TIMEOUT);
        conveyorMotor.configPeakOutputReverse(-1, Constants.Conveyor.TIMEOUT);

        conveyorMotor.configAllowableClosedloopError(Constants.Conveyor.PID_LOOP_IDX,
                Constants.Conveyor.TOLERANCE, Constants.Conveyor.TIMEOUT);

        conveyorMotor.config_kF(Constants.Conveyor.PID_LOOP_IDX, Constants.Conveyor.CONVEYOR_F_COEFF,
                Constants.Conveyor.TIMEOUT);
        conveyorMotor.config_kP(Constants.Conveyor.PID_LOOP_IDX, Constants.Conveyor.CONVEOR_P_COEFF,
                Constants.Conveyor.TIMEOUT);
        conveyorMotor.config_kI(Constants.Conveyor.PID_LOOP_IDX, Constants.Conveyor.CONVEYOR_I_COEFF,
                Constants.Conveyor.TIMEOUT);
        conveyorMotor.config_kD(Constants.Conveyor.PID_LOOP_IDX, Constants.Conveyor.CONVEYOR_D_COEFF,
                Constants.Conveyor.TIMEOUT);
    }

    public void setSpeed(double speed) {
        conveyorMotor.set(ControlMode.PercentOutput, speed);
    }

    public void moveToPosition(double pos) {
        conveyorMotor.set(ControlMode.Position, pos);
    }

    public void moveByPosition(double amount) {
        conveyorMotor.set(ControlMode.Position, conveyorMotor.getSelectedSensorPosition() + amount);
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

    public void addBall(BALL_COLORS newColor) {
        currentBalls.add(newColor);
    }

    public boolean hasBall() {
        return currentBalls.isEmpty();
    }

    public void removeBall() {
        currentBalls.remove();
    }

    public BALL_COLORS topBall() {
        return currentBalls.peek();
    }

}
