package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.utils.HoodPIDController;

public class Shooter extends SubsystemBase {

        private final WPI_TalonFX shooterMotor1;
        private final WPI_TalonFX shooterMotor2;

        private final WPI_TalonFX hoodMotor;

        private final DigitalInput hoodSwitch;

        private final HoodPIDController hoodController;

        public Shooter() {
                ShuffleboardTab tab = Shuffleboard.getTab("Shooter");

                shooterMotor1 = new WPI_TalonFX(RobotMap.ShooterHardware.SHOOTER_MOTOR_1);
                shooterMotor2 = new WPI_TalonFX(RobotMap.ShooterHardware.SHOOTER_MOTOR_2);
                hoodMotor = new WPI_TalonFX(RobotMap.ShooterHardware.HOOD_MOTOR);
                hoodSwitch = new DigitalInput(RobotMap.ShooterHardware.HOOD_SWITCH);

                initMotors();

                shooterMotor1.setNeutralMode(NeutralMode.Coast);
                shooterMotor2.setNeutralMode(NeutralMode.Coast);
                hoodMotor.setNeutralMode(NeutralMode.Brake);

                shooterMotor1.setInverted(TalonFXInvertType.Clockwise);

                hoodMotor.setInverted(TalonFXInvertType.Clockwise);

                setHoodEncoderPos(0);

                shooterMotor2.follow(shooterMotor1);
                shooterMotor2.setInverted(TalonFXInvertType.OpposeMaster);

                hoodController = new HoodPIDController(
                                Constants.ShooterConstants.HOOD_P_COEFF,
                                Constants.ShooterConstants.HOOD_I_COEFF,
                                Constants.ShooterConstants.HOOD_D_COEFF);

                hoodController.setTolerance(Constants.ShooterConstants.HOOD_TOLERANCE);
                
                hoodController.setIntegratorRange(
                                Constants.ShooterConstants.HOOD_INTEGRAL_MIN,
                                Constants.ShooterConstants.HOOD_INTEGRAL_MAX);

                tab.getLayout("Shooter Wheel", BuiltInLayouts.kList)
                                .withSize(2, 2)
                                .withPosition(0, 0)
                                .addNumber("Left Shooter Velocity", () -> shooterMotor1.getSelectedSensorVelocity());

                tab.getLayout("Shooter Wheel", BuiltInLayouts.kList)
                                .withSize(2, 2)
                                .withPosition(0, 0)
                                .addNumber("Right Shooter Velocity", () -> shooterMotor2.getSelectedSensorVelocity());

                tab.getLayout("Hood", BuiltInLayouts.kList)
                                .withSize(2, 2)
                                .withPosition(2, 0)
                                .addNumber("Position Target", () -> hoodController.getSetpoint());

                tab.getLayout("Hood", BuiltInLayouts.kList)
                                .withSize(2, 2)
                                .withPosition(2, 0)
                                .addNumber("Current Position", () -> hoodMotor.getSelectedSensorPosition());

                tab.getLayout("Hood", BuiltInLayouts.kList)
                                .withSize(2, 2)
                                .withPosition(2, 0)
                                .addNumber("Current Error", () -> hoodController.getPositionError());
        }

        public void setSpeed(double speed) {
                shooterMotor1.set(ControlMode.PercentOutput, speed);
        }

        public void setVelocity(double rpm) {
                shooterMotor1.set(ControlMode.Velocity, rpm * 2048.0 / 600.0);
                System.out.println("Target:" + rpm * 2048.0 / 600.0);
        }

        public boolean atVelocity() {
                return Math.abs(shooterMotor1.getClosedLoopError()) <= Constants.ShooterConstants.SHOOTER_VELO_TOLERANCE;
        }

        public void hoodToPos(double pos) {
                hoodController.reset();
                hoodController.setSetpoint(pos);
        }

        public boolean hoodAtPos() {
                return hoodController.atSetpoint();
        }

        public void setHoodSpeed(double speed) {
                hoodMotor.set(ControlMode.PercentOutput, speed);
        }

        public double getHoodPos() {
                return hoodMotor.getSelectedSensorPosition();
        }

        public void hoodToDegree(double degree) {
                hoodToPos((Constants.ShooterConstants.HOOD_LOWER_BOUND - degree)
                                * Constants.ShooterConstants.HOOD_DEGREES_TO_TICKS);
        }

        public boolean getHoodSwitchState() {
                return hoodSwitch.get();
        }

        public void setHoodEncoderPos(double pos) {
                hoodMotor.setSelectedSensorPosition(pos);
        }

        public void initMotors() {
                shooterMotor1.configFactoryDefault();
                hoodMotor.configFactoryDefault();

                shooterMotor1.configNeutralDeadband(0.001);

                shooterMotor1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                                Constants.ShooterConstants.PID_LOOP_IDX,
                                Constants.ShooterConstants.TIMEOUT);

                shooterMotor2.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                                Constants.ShooterConstants.PID_LOOP_IDX,
                                Constants.ShooterConstants.TIMEOUT);

                shooterMotor1.configNominalOutputForward(0, Constants.ShooterConstants.TIMEOUT);
                shooterMotor1.configNominalOutputReverse(0, Constants.ShooterConstants.TIMEOUT);
                shooterMotor1.configPeakOutputForward(1, Constants.ShooterConstants.TIMEOUT);
                shooterMotor1.configPeakOutputReverse(-1, Constants.ShooterConstants.TIMEOUT);

                shooterMotor1.config_kF(Constants.ShooterConstants.PID_LOOP_IDX, Constants.ShooterConstants.SHOOTER_F_COEFF,
                                Constants.ShooterConstants.TIMEOUT);
                shooterMotor1.config_kP(Constants.ShooterConstants.PID_LOOP_IDX, Constants.ShooterConstants.SHOOTER_P_COEFF,
                                Constants.ShooterConstants.TIMEOUT);
                shooterMotor1.config_kI(Constants.ShooterConstants.PID_LOOP_IDX, Constants.ShooterConstants.SHOOTER_I_COEFF,
                                Constants.ShooterConstants.TIMEOUT);
                shooterMotor1.config_kD(Constants.ShooterConstants.PID_LOOP_IDX, Constants.ShooterConstants.SHOOTER_D_COEFF,
                                Constants.ShooterConstants.TIMEOUT);

                hoodMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                                Constants.ShooterConstants.PID_LOOP_IDX,
                                Constants.ShooterConstants.TIMEOUT);

                hoodMotor.configNominalOutputForward(0, Constants.ShooterConstants.TIMEOUT);
                hoodMotor.configNominalOutputReverse(0, Constants.ShooterConstants.TIMEOUT);
                hoodMotor.configPeakOutputForward(1, Constants.ShooterConstants.TIMEOUT);
                hoodMotor.configPeakOutputReverse(-1, Constants.ShooterConstants.TIMEOUT);
        }

        @Override
        public void periodic() {
                setHoodSpeed(hoodController.calculate(getHoodPos()));
                //if (hoodController.getPositionError() * hoodController.())
        }
}
