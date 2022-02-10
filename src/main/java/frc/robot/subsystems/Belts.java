package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

import frc.robot.Constants;
public class Belts extends SubsystemBase {
private final TalonFX beltMotor1;
 
    public Belts()
    {
    beltMotor1 = new TalonFX(RobotMap.Belts.BELT_MOTOR_1);
    }    
    public void runBelts(double speed)
    {
        beltMotor1.set( ControlMode.PercentOutput , speed);
    }

    public void stopAllBelts()
    {
        beltMotor1.set( ControlMode.PercentOutput , 0);

    }
    
}
