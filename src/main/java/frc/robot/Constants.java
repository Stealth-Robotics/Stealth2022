package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class Constants {

        public static final class DriveBase {
                /**
                 * TODO: Fix TRACKWIDTH & WHEELBASE Based On Production Robot
                 * (In Meters)
                 */

                public static final double TRACKWIDTH = 0.0; // Distance From Left Wheel Middle To Right Wheel Middle
                public static final double WHEELBASE = 0.0; // Distance From Front Wheel Middle To Back Wheel Middle

                /**
                 * TODO: Fix Module Steer Offsets Based On Production Robot
                 * (In Degrees)
                 */

                public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0.0);
                public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0.0);
                public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0.0);
                public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0.0);

                public static final double MAX_VOLTAGE = 12.0;

                public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
                                SdsModuleConfigurations.MK4_L2.getDriveReduction() *
                                SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;

                public static final double MAX_ACCELERATION_METERS_PER_SECOND = 1.0;

                public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
                                Math.hypot(TRACKWIDTH / 2.0, WHEELBASE / 2.0);

                public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = Math.PI;

                public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
                                // Front left
                                new Translation2d(TRACKWIDTH / 2.0, WHEELBASE / 2.0),
                                // Front right
                                new Translation2d(TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
                                // Back left
                                new Translation2d(-TRACKWIDTH / 2.0, WHEELBASE / 2.0),
                                // Back right
                                new Translation2d(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0));

                public static final double X_P_CONTROLLER = 0.1;
                public static final double X_I_CONTROLLER = 0.1;
                public static final double X_D_CONTROLLER = 0.1;

                public static final double Y_P_CONTROLLER = 0.1;
                public static final double Y_I_CONTROLLER = 0.1;
                public static final double Y_D_CONTROLLER = 0.1;

                public static final double THETA_P_CONTROLLER = 0.1;
                public static final double THETA_I_CONTROLLER = 0.1;
                public static final double THETA_D_CONTROLLER = 0.1;

                public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(
                                MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                                MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED);

                public static final TrajectoryConfig CONFIG = new TrajectoryConfig(
                                Constants.DriveBase.MAX_VELOCITY_METERS_PER_SECOND,
                                Constants.DriveBase.MAX_ACCELERATION_METERS_PER_SECOND)
                                                .setKinematics(Constants.DriveBase.DRIVE_KINEMATICS);
        }

        public static final class Limelight {
                public static final double TARGET_HEIGHT = 2.49; // meters
                public static final double CROSSHAIR_HEIGHT = 2.27; // meters | old: 2.23, old: 2.3876 | height of
                                                                    // center
                public static final double CAMERA_HEIGHT = 0.65405; // in meters
                public static final double CAMERA_ANGLE = 27 * (Math.PI / 180); // in radians

        }

        public static final class Shooter {
                public static final double SHOOTER_F_COEFF = 1023.0 / 20660.0;
                public static final double SHOOTER_P_COEFF = 0.1;
                public static final double SHOOTER_I_COEFF = 0.001;
                public static final double SHOOTER_D_COEFF = 5;

                public static final int TIMEOUT = 30; // In Milliseconds

                public static final int PID_LOOP_IDX = 0;

                public static final double HOOD_P = 0.1;
                public static final double HOOD_I = 0.001;
                public static final double HOOD_D = 5;

                public static final double HOOD_LOWER_BOUND = 0;
                public static final double HOOD_UPPER_BOUND = 100;
        }

        public static final class Conveyer {
                public enum BALL_COLORS {
                        RED,
                        BLUE,
                }
        }
}
