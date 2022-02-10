package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Intake extends SubsystemBase {

    TalonFX intake;

    /**
     * Creates a new Intake.
     */
    public Intake() {
        intake = new TalonFX(RobotMap.Intake.INTAKE_MOTOR);
        intake.setInverted(true);
    }

    public void setSpeed(double speed) {
        intake.set(ControlMode.PercentOutput, speed);
    }

}