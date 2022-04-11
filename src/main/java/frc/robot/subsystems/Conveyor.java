package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Conveyor extends SubsystemBase {
    private final WPI_TalonFX conveyorMotor;
    private final DigitalInput beamBreak;
    private double target = 0;
    private double startingPosition = 0;

    public Conveyor() {
        conveyorMotor = new WPI_TalonFX(RobotMap.ConveyorHardware.CONVEYER_MOTOR);
        beamBreak = new DigitalInput(RobotMap.ConveyorHardware.BEAM_BREAK);

        conveyorMotor.configFactoryDefault();
        conveyorMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                Constants.ConveyorConstants.PID_LOOP_IDX, Constants.ConveyorConstants.TIMEOUT);

        conveyorMotor.configNominalOutputForward(0, Constants.ConveyorConstants.TIMEOUT);
        conveyorMotor.configNominalOutputReverse(0, Constants.ConveyorConstants.TIMEOUT);
        conveyorMotor.configPeakOutputForward(0.4, Constants.ConveyorConstants.TIMEOUT);
        conveyorMotor.configPeakOutputReverse(-0.4, Constants.ConveyorConstants.TIMEOUT);

        conveyorMotor.setInverted(true);

    }

    public void moveByAmount(double amount) {
        startingPosition = getConveyorPosition();
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
        return target >= startingPosition ? getConveyorPosition() >= target : getConveyorPosition() <= target;
    }

    @Override
    public void periodic() {

        if (!atPosition() && (target > startingPosition) && getSpeed() != 0.4) {
            setSpeed(0.4);
        }

        else if (!atPosition() && (target < startingPosition) && getSpeed() != -0.4) {
            setSpeed(-0.4);
        }

        else if (atPosition() && getSpeed() != 0.0) {
            setSpeed(0.0);
        }
    }
}