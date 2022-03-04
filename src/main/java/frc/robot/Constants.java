package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {

        public static final class DriveBaseConstants {
                /**
                 * TODO: Fix TRACKWIDTH & WHEELBASE Based On Production Robot
                 * (In Meters)
                 */

                public static final double TRACKWIDTH = Units.inchesToMeters(18.5); // Distance From Left Wheel
                                                                                    // Middle To Right Wheel Middle
                public static final double WHEELBASE = Units.inchesToMeters(25.5); // Distance From Front Wheel
                                                                                   // Middle To Back Wheel Middle

                /**
                 * TODO: Fix Module Steer Offsets Based On Production Robot
                 * (In Degrees)
                 */

                public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(340.6585693359375);
                public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(350.244140625);
                public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(196.435546875);
                public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(332.92694091796875);

                public static final double MAX_VOLTAGE = 12.0;

                public static final double X_P_COEFF = 3;
                public static final double Y_P_COEFF = 3;
                public static final double THETA_P_COEFF = 4;

                public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
                                SdsModuleConfigurations.MK4_L2.getDriveReduction() *
                                SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;

                public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3.0;

                public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
                                Math.hypot(TRACKWIDTH / 2.0, WHEELBASE / 2.0);

                public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = Math.PI;

                public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(
                                MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                                MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED);

                public static final TrajectoryConfig MAX_SPEED_CONFIG = new TrajectoryConfig(
                                MAX_VELOCITY_METERS_PER_SECOND,
                                MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

                public static final TrajectoryConfig MEDIUM_SPEED_CONFIG = new TrajectoryConfig(
                                0.6 * MAX_VELOCITY_METERS_PER_SECOND,
                                0.6 * MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

                public static final TrajectoryConfig SLOW_SPEED_CONFIG = new TrajectoryConfig(
                                0.3 * MAX_VELOCITY_METERS_PER_SECOND,
                                0.3 * MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

                public static final double SLOWMODE_MULTIPLIER = 1 / 4;

                public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
                                // Front left
                                new Translation2d(TRACKWIDTH / 2.0, WHEELBASE / 2.0),
                                // Front right
                                new Translation2d(TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
                                // Back left
                                new Translation2d(-TRACKWIDTH / 2.0, WHEELBASE / 2.0),
                                // Back right
                                new Translation2d(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0));

                public static final double ALIGN_P_COEFF = 0.08;
                public static final double ALIGN_I_COEFF = 0.000;
                public static final double ALIGN_D_COEFF = 0.0010;

                public static final double ALIGN_TOLERANCE = 7;
        }

        public static final class LimelightConstants {
                public static final double MOUNTED_ANGLE = 35; // Degrees
                public static final double LENS_HEIGHT = 36.73; // Inches
                public static final double TARGET_HEIGHT = 103; // Inches

                public static final double LENS_TO_SHOOTER = 11.78; // Inches

        }

        public static final class ShooterConstants {
                public static final double SHOOTER_F_COEFF = 1023.0 / 20660.0;
                public static final double SHOOTER_P_COEFF = 0.07;
                public static final double SHOOTER_I_COEFF = 0.000;
                public static final double SHOOTER_D_COEFF = 0;

                public static final double SHOOTER_VELO_TOLERANCE = 5;

                public static final int TIMEOUT = 30; // In Milliseconds

                public static final int PID_LOOP_IDX = 0;

                public static final double HOOD_P_COEFF = 0.0001;
                public static final double HOOD_I_COEFF = 0.00004;
                public static final double HOOD_D_COEFF = 0.0;

                public static final double HOOD_TOLERANCE = 1000;

                public static final double HOOD_INTEGRAL_MIN = -0.2;
                public static final double HOOD_INTEGRAL_MAX = 0.2;

                public static final double HOOD_LOWER_BOUND = 82;
                public static final double HOOD_UPPER_BOUND = 58;

                public static final double HOOD_DEGREES_TO_TICKS = 2184.533;
        }

        public static final class ConveyorConstants {
                public enum BALL_COLORS {
                        RED,
                        BLUE,
                        UNKNOWN,
                        EMPTY
                }

                public static final int TIMEOUT = 30; // In Milliseconds

                public static final int PID_LOOP_IDX = 0;

                public static final double SHOOT_CONVEYOR_STEP = 20000;
                public static final double INDEXING_STEP = 5000;
        }

        public static final class IOConstants {
                public static final int DRIVE_JOYSTICK_PORT = 0;
                public static final int MECH_GAMEPAD_PORT = 1;
                public static final int DRIVER_STATION_PORT = 2;

                public static final double DRIVE_JOYSTICK_DEADZONE = 0.06;
        }

        public static final class CANdleSystemConstants {
                public static final int TIMEOUT = 30;
        }
        public static final class ClimberConstants {
                public static final double MAX_THRESHOLD = 140000;
                public static final double MIN_THRESHOLD = -3000;
                public static final double MAX_SPEED = 0.6;
        }
}
