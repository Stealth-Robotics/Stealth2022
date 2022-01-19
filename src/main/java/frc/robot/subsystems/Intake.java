
package frc.robot.subsystems;
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
         intake = new TalonFX(RobotMap.Subsystems.INTAKE_MOTOR);
         intake.setInverted(true);
    }

    /**
     * Sets the intake speed to 1
     */
    public void run()
    {
        intake.set(ControlMode.PercentOutput, 1);    
    }

    /**
     * Stops the intake
     */
    public void stopIntake()
    {
        intake.set(ControlMode.PercentOutput, 0);
        
    }

    /**
     * Reverses the intake
     */
    public void reverse()
    {
        
        intake.set(ControlMode.PercentOutput, -0.5);
    }

    /**
     * Toggles the deployment state of the intake
     */
   
    
}