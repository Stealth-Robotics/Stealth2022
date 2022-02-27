package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class CANdleSystem extends SubsystemBase {
    private final CANdle leftCANdle;
    private final CANdle rightCANdle;

    public CANdleSystem()
    {
        leftCANdle = new CANdle(RobotMap.CANdleSubsystem.LEFT_CANDLE);
        rightCANdle = new CANdle(RobotMap.CANdleSubsystem.RIGHT_CANDLE);

        CANdleConfiguration config = new CANdleConfiguration();
        config.brightnessScalar = 1.0;
        config.statusLedOffWhenActive = true;
        config.disableWhenLOS = true;
        
        leftCANdle.configAllSettings(config, Constants.CANdleSubsystem.TIMEOUT);
        rightCANdle.configAllSettings(config, Constants.CANdleSubsystem.TIMEOUT);
    }

    public void blink()
    {
        leftCANdle.animate(new StrobeAnimation(0, 255, 0, 0, 98.0/256.0, 0));
        rightCANdle.animate(new StrobeAnimation(0, 255, 0, 0, 98.0/256.0, 0));
    }

    public void turnOff()
    {
        leftCANdle.setLEDs(0, 0, 0);
        rightCANdle.setLEDs(0, 0, 0);
    }
}
