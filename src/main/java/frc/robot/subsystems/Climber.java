package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Climber extends SubsystemBase {

    private final WPI_TalonFX climberMotor1;
    private final WPI_TalonFX climberMotor2;

    private final Solenoid pivotPistons;

    public Climber() {
        climberMotor1 = new WPI_TalonFX(RobotMap.Climber.CLIMBER_MOTOR_1);
        climberMotor2 = new WPI_TalonFX(RobotMap.Climber.CLIMBER_MOTOR_2);

        climberMotor1.setNeutralMode(NeutralMode.Brake);
        climberMotor2.setNeutralMode(NeutralMode.Brake);

        climberMotor1.setInverted(TalonFXInvertType.Clockwise); //TODO: Needs to be checked
        climberMotor2.follow(climberMotor1);
        climberMotor2.setInverted(TalonFXInvertType.FollowMaster);

        pivotPistons = new Solenoid(RobotMap.Pneumatics.PCM,
         RobotMap.Pneumatics.CLIMBER_DEPLOY_PCM_CHANNEL);
    }

    public void setSpeed(double speed) {
        climberMotor1.set(ControlMode.PercentOutput, speed);
    }
    public void togglePivotPistons() {
        pivotPistons.toggle();;
    }
   
}