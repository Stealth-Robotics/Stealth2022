package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

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

        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
                Math.hypot(TRACKWIDTH / 2.0, WHEELBASE / 2.0);

        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
                // Front left
                new Translation2d(TRACKWIDTH / 2.0, WHEELBASE / 2.0),
                // Front right
                new Translation2d(TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
                // Back left
                new Translation2d(-TRACKWIDTH / 2.0, WHEELBASE / 2.0),
                // Back right
                new Translation2d(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0));
    }
}
