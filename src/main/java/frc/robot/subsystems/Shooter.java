package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase {

        private final TalonFX shooterMotor1;
        private final TalonFX shooterMotor2;

        private final TalonFX hoodMotor;

        private final PIDController hoodController;

        private final DigitalInput hoodSwitch;

        // TODO: Remove After Testing
        private double targetVelo = 0;

        public Shooter() {

                shooterMotor1 = new TalonFX(RobotMap.Shooter.SHOOTER_MOTOR_1);
                shooterMotor2 = new TalonFX(RobotMap.Shooter.SHOOTER_MOTOR_2);
                hoodMotor = new TalonFX(RobotMap.Shooter.HOOD_MOTOR);
                hoodSwitch = new DigitalInput(RobotMap.Shooter.HOOD_SWITCH);

                initMotors();

                shooterMotor1.setNeutralMode(NeutralMode.Coast);
                shooterMotor2.setNeutralMode(NeutralMode.Coast);
                hoodMotor.setNeutralMode(NeutralMode.Brake);

                hoodController = new PIDController(
                                Constants.Shooter.HOOD_P,
                                Constants.Shooter.HOOD_I,
                                Constants.Shooter.HOOD_D);

                hoodController.setTolerance(5);

                shooterMotor1.setInverted(TalonFXInvertType.Clockwise);

                hoodMotor.setInverted(TalonFXInvertType.CounterClockwise);

                setHoodEncoderPos(0);

                shooterMotor2.follow(shooterMotor1);
                shooterMotor2.setInverted(TalonFXInvertType.OpposeMaster);
        }

        public void setSpeed(double speed) {
                shooterMotor1.set(ControlMode.PercentOutput, speed);
        }

        public void setVelocity(double proportion) {
                targetVelo = proportion * 2000.0 * 2048.0 / 600.0;
                shooterMotor1.set(ControlMode.Velocity, proportion * 2000.0 * 2048.0 / 600.0);
        }

        public void hoodToPos(double pos) {
                if (pos >= Constants.Shooter.HOOD_LOWER_BOUND
                                && pos <= Constants.Shooter.HOOD_UPPER_BOUND) {
                        hoodController.setSetpoint(pos);
                        hoodController.reset();
                }
        }

        public void setHoodSpeed(double speed) {
                hoodMotor.set(ControlMode.PercentOutput, speed);
        }

        public double getHoodPos() {
                return hoodMotor.getSelectedSensorPosition();
        }

        public boolean getHoodSwitchState() {
                return hoodSwitch.get();
        }

        public void setHoodEncoderPos(double pos) {
                hoodMotor.setSelectedSensorPosition(pos);
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

                hoodMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
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

                System.out.println("Current Hood Pos: " + hoodMotor.getSelectedSensorPosition());

                setHoodSpeed(hoodController.calculate(hoodMotor.getSelectedSensorPosition()));
        }
}
