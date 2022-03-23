package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.utils.HoodPIDController;

public class Climber extends SubsystemBase {

    private final WPI_TalonFX climberMotor1;
    private final WPI_TalonFX climberMotor2;

    private final Solenoid pivotPistons;

    private final HoodPIDController climbController;

    public Climber() {
        ShuffleboardTab tab = Shuffleboard.getTab("Climber");

        climberMotor1 = new WPI_TalonFX(RobotMap.ClimberHardware.CLIMBER_MOTOR_1);
        climberMotor2 = new WPI_TalonFX(RobotMap.ClimberHardware.CLIMBER_MOTOR_2);

        climberMotor1.setNeutralMode(NeutralMode.Brake);
        climberMotor2.setNeutralMode(NeutralMode.Brake);

        climberMotor1.setInverted(TalonFXInvertType.CounterClockwise); // TODO: Needs to be checked
        climberMotor2.follow(climberMotor1);
        climberMotor2.setInverted(TalonFXInvertType.FollowMaster);

        pivotPistons = new Solenoid(RobotMap.Pneumatics.PCM, RobotMap.Pneumatics.PCM_TYPE,
                RobotMap.Pneumatics.CLIMBER_DEPLOY_PCM_CHANNEL);

        tab.getLayout("Climber Position", BuiltInLayouts.kList)
                .withPosition(0, 0)
                .withSize(2, 1)
                .addNumber("Current Position", () -> getClimberPosition());

        climberMotor1.setSelectedSensorPosition(0);

        // automated climb PID
///*
        climbController = new HoodPIDController(
                Constants.ClimberConstants.CLIMB_P_COEFF,
                Constants.ClimberConstants.CLIMB_I_COEFF,
                Constants.ClimberConstants.CLIMB_D_COEFF);

        climbController.setTolerance(Constants.ClimberConstants.CLIMB_TOLERANCE);

        climbController.setIntegratorRange(
                Constants.ClimberConstants.CLIMB_INTEGRAL_MIN,
                Constants.ClimberConstants.CLIMB_INTEGRAL_MAX);
        climberMotor1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                Constants.ClimberConstants.PID_LOOP_IDX,
                Constants.ClimberConstants.TIMEOUT);

        climberMotor1.configNominalOutputForward(0, Constants.ShooterConstants.TIMEOUT);
        climberMotor1.configNominalOutputReverse(0, Constants.ShooterConstants.TIMEOUT);
        climberMotor1.configPeakOutputForward(0.5, Constants.ShooterConstants.TIMEOUT);
        climberMotor1.configPeakOutputReverse(-0.5, Constants.ShooterConstants.TIMEOUT);
//*/
    }

    public void setSpeed(double speed) {
        climberMotor1.set(ControlMode.PercentOutput, speed);
    }

    public void togglePivotPistons() {
        pivotPistons.set(!pivotPistons.get());
    }

    public void movePisitons(boolean setState) {
        pivotPistons.set(setState);
    }

    public double getClimberPosition() {
        return climberMotor1.getSelectedSensorPosition();
    }

    public double getClimberPower() {
        return climberMotor1.getMotorOutputPercent();
    }

    public void resetClimberEncoder() {
        climberMotor1.setSelectedSensorPosition(0);
    }
///*
    @Override
    public void periodic() {
        setSpeed(climbController.calculate(getClimberPosition()));
    }

    public void climberToPos(double pos) {
        climbController.reset();
        climbController.setSetpoint(pos);
    }

    public boolean climberAtPos() {
        return climbController.atSetpoint();
    }
    //*/

}