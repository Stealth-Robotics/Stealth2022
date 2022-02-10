package frc.robot;

public final class RobotMap {
    public static final class DriveBase {

        /**
         * TODO: Set CAN IDs Based On Production Robot
         */

        public static final int FRONT_LEFT_MODULE_DRIVE = 0;
        public static final int FRONT_LEFT_MODULE_STEERING = 1;
        public static final int FRONT_LEFT_MODULE_ENCODER = 2;

        public static final int FRONT_RIGHT_MODULE_DRIVE = 3;
        public static final int FRONT_RIGHT_MODULE_STEERING = 4;
        public static final int FRONT_RIGHT_MODULE_ENCODER = 5;

        public static final int BACK_LEFT_MODULE_DRIVE = 6;
        public static final int BACK_LEFT_MODULE_STEERING = 7;
        public static final int BACK_LEFT_MODULE_ENCODER = 8;

        public static final int BACK_RIGHT_MODULE_DRIVE = 9;
        public static final int BACK_RIGHT_MODULE_STEERING = 10;
        public static final int BACK_RIGHT_MODULE_ENCODER = 11;

        public static final int PIGEON_IMU = 12;

    }

    public static final class Intake {

        /**
         * TODO: Set CAN IDs Based On Production Robot
         */

        public static final int INTAKE_MOTOR = 1;
    }

    public static final class Shooter {

        /**
         * TODO: Set CAN IDs Based On Production Robot
         */

        public static final int SHOOTER_MOTOR_1 = 2;
        public static final int SHOOTER_MOTOR_2 = 2;
        public static final int HOOD_MOTOR = 3;
    }

    public static final class Conveyor {

        /**
         * TODO: Set CAN IDs Based On Production Robot
         */

        public static final int CONVEYER_MOTOR = 4;
    }

}
