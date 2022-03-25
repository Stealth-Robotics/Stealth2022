package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

public final class RobotMap {
    public static final class DriveBaseHardware {

        public static final int FRONT_LEFT_MODULE_DRIVE = 13;
        public static final int FRONT_LEFT_MODULE_STEERING = 23;
        public static final int FRONT_LEFT_MODULE_ENCODER = 33;

        public static final int FRONT_RIGHT_MODULE_DRIVE = 14;
        public static final int FRONT_RIGHT_MODULE_STEERING = 24;
        public static final int FRONT_RIGHT_MODULE_ENCODER = 34;

        public static final int BACK_LEFT_MODULE_DRIVE = 11;
        public static final int BACK_LEFT_MODULE_STEERING = 21;
        public static final int BACK_LEFT_MODULE_ENCODER = 31;

        public static final int BACK_RIGHT_MODULE_DRIVE = 12;
        public static final int BACK_RIGHT_MODULE_STEERING = 22;
        public static final int BACK_RIGHT_MODULE_ENCODER = 32;

        public static final int PIGEON_IMU = 45;

    }

    public static final class IntakeHardware {

        public static final int INTAKE_MOTOR = 1;
    }

    public static final class ShooterHardware {

        public static final int SHOOTER_MOTOR_1 = 8;
        public static final int SHOOTER_MOTOR_2 = 9;
        public static final int HOOD_MOTOR = 7;
        public static final int HOOD_SWITCH = 4;
    }

    public static final class ClimberHardware {
        public static final int CLIMBER_MOTOR_1 = 5;
        public static final int CLIMBER_MOTOR_2 = 6;

    }

    public static final class Conveyor {

        public static final int CONVEYER_MOTOR = 2;
        public static final int BEAM_BREAK = 0;
    }

    public static final class CANdleSystem {
        public static final int LEFT_CANDLE = 45;
        public static final int RIGHT_CANDLE = 46;
    }

    public static final class Pneumatics {
        public static final int PCM = 41;
        public static final PneumaticsModuleType PCM_TYPE = PneumaticsModuleType.REVPH;
        public static final int INTAKE_DEPLOY_PCM_CHANNEL = 0;
        public static final int CLIMBER_DEPLOY_PCM_CHANNEL = 1;
    }

}
