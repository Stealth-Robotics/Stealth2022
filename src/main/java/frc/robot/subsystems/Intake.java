package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Intake extends SubsystemBase {

    private final WPI_TalonFX intake;

    private final Solenoid deployPistons;

    /**
     * Creates a new Intake.
     */
    public Intake() {
        intake = new WPI_TalonFX(RobotMap.Intake.INTAKE_MOTOR);
        intake.setInverted(true);
        deployPistons = new Solenoid(RobotMap.Pneumatics.PCM, RobotMap.Pneumatics.INTAKE_DEPLOY_PCM_CHANNEL);
    }

    public void setSpeed(double speed) {
        intake.set(ControlMode.PercentOutput, speed);
    }

    public void deploy() {
        deployPistons.set(true);
    }

    public void unDeploy() {
        deployPistons.set(false);
    }

    public void toggle() {
        deployPistons.toggle();
    }

}