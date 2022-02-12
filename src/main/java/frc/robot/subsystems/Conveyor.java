package frc.robot.subsystems;

import java.util.Queue;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Conveyor extends SubsystemBase {
    private final TalonFX conveyorMotor;
    private final DigitalInput beamBreak;

    private enum BallColors {
        RED,
        BLUE,
    }

    private Queue<BallColors> currentBalls;

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

    public void addBall(BallColors newColor) {
        currentBalls.add(newColor);
    }

    public boolean hasBall() {
        return currentBalls.isEmpty();
    }

    public void removeBall() {
        currentBalls.remove();
    }

    public BallColors topBall() {
        return currentBalls.peek();
    }

}
