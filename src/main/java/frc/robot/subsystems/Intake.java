package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Intake extends SubsystemBase {

    private final WPI_TalonFX intake;

    final Solenoid deployPistons;

    public Intake() {
        intake = new WPI_TalonFX(RobotMap.IntakeHardware.INTAKE_MOTOR);
        intake.setInverted(true);
        deployPistons = new Solenoid(RobotMap.Pneumatics.PCM, RobotMap.Pneumatics.PCM_TYPE,
                RobotMap.Pneumatics.INTAKE_DEPLOY_PCM_CHANNEL);
        intake.setStatusFramePeriod(1, 233);
        intake.setStatusFramePeriod(2, 233);
    }

    /**
     * Sets intake to a given speed.
     * 
     * @param speed The given speed to set the intake to.
     */
    public void setSpeed(double speed) {
        intake.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Deploys intake pistons down.
     */
    public void deploy() {
        deployPistons.set(true);
    }

    /**
     * Undeploys intake pistons up.
     */
    public void unDeploy() {
        deployPistons.set(false);
    }

    /**
     * Toggles pistons to the opposite of their current state.
     */
    public void toggle() {
        deployPistons.set(!deployPistons.get());
    }

}