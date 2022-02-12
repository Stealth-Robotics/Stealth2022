package frc.robot.subsystems;

import java.util.Queue;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.Constants.Conveyer.*;

public class Conveyor extends SubsystemBase {
    private final TalonFX conveyorMotor;
    private final DigitalInput beamBreak;

    private Queue<BALL_COLORS> currentBalls;

    public Conveyor() {
        conveyorMotor = new TalonFX(RobotMap.Conveyor.CONVEYER_MOTOR);
        beamBreak = new DigitalInput(RobotMap.Conveyor.BEAM_BREAK);
    }

    public void setSetSpeed(double speed) {
        conveyorMotor.set(ControlMode.PercentOutput, speed);
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
