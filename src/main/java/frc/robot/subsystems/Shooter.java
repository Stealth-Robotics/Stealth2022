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

public class Shooter extends SubsystemBase {

        private final WPI_TalonFX shooterMotor1;
        private final WPI_TalonFX shooterMotor2;

        private final WPI_TalonFX hoodMotor;

        private final DigitalInput hoodSwitch;

        private final PIDController hoodController;

        public Shooter() {
                ShuffleboardTab tab = Shuffleboard.getTab("Shooter");

                shooterMotor1 = new WPI_TalonFX(RobotMap.Shooter.SHOOTER_MOTOR_1);
                shooterMotor2 = new WPI_TalonFX(RobotMap.Shooter.SHOOTER_MOTOR_2);
                hoodMotor = new WPI_TalonFX(RobotMap.Shooter.HOOD_MOTOR);
                hoodSwitch = new DigitalInput(RobotMap.Shooter.HOOD_SWITCH);

                initMotors();

                shooterMotor1.setNeutralMode(NeutralMode.Coast);
                shooterMotor2.setNeutralMode(NeutralMode.Coast);
                hoodMotor.setNeutralMode(NeutralMode.Brake);

                shooterMotor1.setInverted(TalonFXInvertType.CounterClockwise);

                hoodMotor.setInverted(TalonFXInvertType.CounterClockwise);

                setHoodEncoderPos(0);

                shooterMotor2.follow(shooterMotor1);
                shooterMotor2.setInverted(TalonFXInvertType.OpposeMaster);

                hoodController = new PIDController(
                                Constants.Shooter.HOOD_P_COEFF,
                                Constants.Shooter.HOOD_I_COEFF,
                                Constants.Shooter.HOOD_D_COEFF);

                hoodController.setTolerance(Constants.Shooter.HOOD_TOLERANCE);

                tab.getLayout("Shooter Wheel", BuiltInLayouts.kList)
                                .withSize(2, 2)
                                .withPosition(0, 0)
                                .addNumber("Velocity Target", () -> shooterMotor1.getClosedLoopTarget());

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
        }

        public void setSpeed(double speed) {
                shooterMotor1.set(ControlMode.PercentOutput, speed);
        }

        public void setVelocity(double rpm) {
                shooterMotor1.set(ControlMode.Velocity, rpm * 2048.0 / 600.0);
        }

        public boolean atVelocity() {
                return Math.abs(shooterMotor1.getClosedLoopError()) <= Constants.Shooter.SHOOTER_VELO_TOLERANCE;
        }

        public void hoodToPos(double pos) {
                hoodMotor.set(ControlMode.Position, pos);
        }

        public boolean hoodAtPos() {
                return Math.abs(hoodMotor.getClosedLoopError(0)) <= Constants.Shooter.HOOD_TOLERANCE;
        }

        public void setHoodSpeed(double speed) {
                hoodMotor.set(ControlMode.PercentOutput, speed);
        }

        public double getHoodPos() {
                return hoodMotor.getSelectedSensorPosition();
        }

        public void hoodToDegree(double degree) {
                hoodToPos((Constants.Shooter.HOOD_LOWER_BOUND - degree)
                                * Constants.Shooter.HOOD_DEGREES_TO_TICKS);
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

                hoodMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                                Constants.Shooter.PID_LOOP_IDX,
                                Constants.Shooter.TIMEOUT);

                hoodMotor.configAllowableClosedloopError(Constants.Shooter.PID_LOOP_IDX,
                                Constants.Shooter.HOOD_TOLERANCE, Constants.Shooter.TIMEOUT);

                hoodMotor.config_kF(Constants.Shooter.PID_LOOP_IDX, Constants.Shooter.HOOD_F_COEFF,
                                Constants.Shooter.TIMEOUT);
                hoodMotor.config_kP(Constants.Shooter.PID_LOOP_IDX, Constants.Shooter.HOOD_P_COEFF,
                                Constants.Shooter.TIMEOUT);
                hoodMotor.config_kI(Constants.Shooter.PID_LOOP_IDX, Constants.Shooter.HOOD_I_COEFF,
                                Constants.Shooter.TIMEOUT);
                hoodMotor.config_kD(Constants.Shooter.PID_LOOP_IDX, Constants.Shooter.HOOD_D_COEFF,
                                Constants.Shooter.TIMEOUT);

                hoodMotor.configNominalOutputForward(0, Constants.Shooter.TIMEOUT);
                hoodMotor.configNominalOutputReverse(0, Constants.Shooter.TIMEOUT);
                hoodMotor.configPeakOutputForward(0.1, Constants.Shooter.TIMEOUT);
                hoodMotor.configPeakOutputReverse(-0.1, Constants.Shooter.TIMEOUT);
        }
}
