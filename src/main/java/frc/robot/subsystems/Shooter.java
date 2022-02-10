package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

    private final TalonFX shooterMotor1;
    private final TalonFX shooterMotor2;
    private final TalonFX shooterHood;

    private final PIDController shootController;
    private final PIDController hoodController;

    public Shooter() {
        shooterMotor1 = new TalonFX(RobotMap.Shooter.SHOOTER_MOTOR_1);
        shooterMotor2 = new TalonFX(RobotMap.Shooter.SHOOTER_MOTOR_2);
        shooterHood = new TalonFX(RobotMap.Shooter.HOOD_MOTOR);

        shooterMotor2.setInverted(true);

        shooterMotor1.setNeutralMode(NeutralMode.Coast);
        shooterMotor2.setNeutralMode(NeutralMode.Coast);

        shootController = new PIDController(
                Constants.Shooter.SHOOTER_P_COEFF,
                Constants.Shooter.SHOOTER_I_COEFF,
                Constants.Shooter.SHOOTER_D_COEFF);

        shootController.enableContinuousInput(-1, 1);

        hoodController = new PIDController(
                Constants.Shooter.HOOD_P_COEFF,
                Constants.Shooter.HOOD_I_COEFF,
                Constants.Shooter.HOOD_D_COEFF);
    }

    public void setSpeed(double speed) {
        shootController.setSetpoint(speed);
    }

    @Override
    public void periodic() {
        double update = shootController.calculate(shooterMotor1.getSelectedSensorVelocity());
        shooterMotor1.set(ControlMode.PercentOutput, update);
        shooterMotor2.set(ControlMode.PercentOutput, update);
    }

}
