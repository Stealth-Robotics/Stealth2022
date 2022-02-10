package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Conveyor extends SubsystemBase {
    private final TalonFX conveyorMotor;

    public Conveyor() {
        conveyorMotor = new TalonFX(RobotMap.Conveyor.CONVEYER_MOTOR);
    }

    public void setSetSpeed(double speed) {
        conveyorMotor.set(ControlMode.PercentOutput, speed);
    }
}
