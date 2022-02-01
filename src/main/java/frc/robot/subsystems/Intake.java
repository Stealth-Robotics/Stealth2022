
package frc.robot.subsystems;
import javax.swing.plaf.basic.BasicComboPopup.InvocationKeyHandler;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Intake extends SubsystemBase 
{

    TalonFX intake;
    /**
     * Creates a new Intake.
     */
    public Intake() 
    {
         intake = new TalonFX(RobotMap.Intake.INTAKE_MOTOR);
         intake.setInverted(true);
    }

    public void setSpeed(double speed)
    {
        assert(-1 <= speed && speed  <= 1);
        intake.set(ControlMode.PercentOutput, speed);
    }

        
}