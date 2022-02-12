package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase {

    private final TalonFX shooterMotor1;
    private final TalonFX shooterMotor2;

    // TODO: Remove After Testing
    private double targetVelo = 0;

    public Shooter() {

        shooterMotor1 = new TalonFX(RobotMap.Shooter.SHOOTER_MOTOR_1);
        shooterMotor2 = new TalonFX(RobotMap.Shooter.SHOOTER_MOTOR_2);

        initMotors();

        shooterMotor1.setNeutralMode(NeutralMode.Coast);
        shooterMotor2.setNeutralMode(NeutralMode.Coast);

        shooterMotor2.setInverted(true);

        shooterMotor2.follow(shooterMotor1); // TODO: CHECK IF IT IS IN REVERSE

        // TODO: Try inverting after the follow if above invert does not work
    }

    public void setSpeed(double speed) {
        shooterMotor1.set(ControlMode.PercentOutput, speed);
    }

    public void setVelocity(double proportion) {
        targetVelo = proportion * 2000.0 * 2048.0 / 600.0;
        shooterMotor1.set(ControlMode.Velocity, proportion * 2000.0 * 2048.0 / 600.0);
    }

    public void initMotors() {
        shooterMotor1.configFactoryDefault();

        shooterMotor1.configNeutralDeadband(0.001);

        shooterMotor1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                Constants.Shooter.PID_LOOP_IDX,
                Constants.Shooter.TIMEOUT);

        shooterMotor2.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                Constants.Shooter.PID_LOOP_IDX,
                Constants.Shooter.TIMEOUT);

        shooterMotor1.configNominalOutputForward(0, Constants.Shooter.TIMEOUT);
        shooterMotor1.configNominalOutputReverse(0, Constants.Shooter.TIMEOUT);
        shooterMotor1.configPeakOutputForward(1, Constants.Shooter.TIMEOUT);
        shooterMotor1.configPeakOutputReverse(-1, Constants.Shooter.TIMEOUT);

        shooterMotor1.config_kF(Constants.Shooter.PID_LOOP_IDX, Constants.Shooter.SHOOTER_F_COEFF,
                Constants.Shooter.TIMEOUT);
        shooterMotor1.config_kP(Constants.Shooter.PID_LOOP_IDX, Constants.Shooter.SHOOTER_P_COEFF,
                Constants.Shooter.TIMEOUT);
        shooterMotor1.config_kI(Constants.Shooter.PID_LOOP_IDX, Constants.Shooter.SHOOTER_I_COEFF,
                Constants.Shooter.TIMEOUT);
        shooterMotor1.config_kD(Constants.Shooter.PID_LOOP_IDX, Constants.Shooter.SHOOTER_D_COEFF,
                Constants.Shooter.TIMEOUT);
    }

    @Override
    public void periodic() {
        // TODO: Remove After Testing
        System.out.println("Target Velo:" + targetVelo
                + ", Shooter 1: " + shooterMotor1.getSelectedSensorVelocity()
                + ", Shooter 2: " + shooterMotor2.getSelectedSensorVelocity());
    }
}
