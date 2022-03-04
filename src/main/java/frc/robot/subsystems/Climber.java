package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Climber extends SubsystemBase {

    private final WPI_TalonFX climberMotor1;
    private final WPI_TalonFX climberMotor2;

    private final Solenoid pivotPistons;

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

    public double getClimberPower()
    {
        return climberMotor1.getMotorOutputPercent();
    }
    public void resetClimberEncoder()
    {
        climberMotor1.setSelectedSensorPosition(0);
    }

}