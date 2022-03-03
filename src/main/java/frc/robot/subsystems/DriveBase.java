package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class DriveBase extends SubsystemBase {

        private final WPI_Pigeon2 pigeon = new WPI_Pigeon2(RobotMap.DriveBase.PIGEON_IMU);

        private final SwerveModule frontLeftModule;
        private final SwerveModule frontRightModule;
        private final SwerveModule backLeftModule;
        private final SwerveModule backRightModule;

        private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
                        Constants.DriveBase.DRIVE_KINEMATICS,
                        getGyroscopeRotation());

        private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

        private final PIDController xController = new PIDController(Constants.DriveBase.X_P_COEFF, 0.001, 0);
        private final PIDController yController = new PIDController(Constants.DriveBase.Y_P_COEFF, 0.001, 0);

        private final ProfiledPIDController thetaController = new ProfiledPIDController(
                        Constants.DriveBase.THETA_P_COEFF, 0.001, 0,
                        Constants.DriveBase.THETA_CONTROLLER_CONSTRAINTS);

        private final HolonomicDriveController pathController = new HolonomicDriveController(
                        xController,
                        yController,
                        thetaController);

        public DriveBase() {
                ShuffleboardTab tab = Shuffleboard.getTab("DriveBase");

                // pigeon.configFactoryDefault();
                setGyroscopeRotation(90);

                frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(0, 0),
                                Mk4SwerveModuleHelper.GearRatio.L2,
                                RobotMap.DriveBase.FRONT_LEFT_MODULE_DRIVE,
                                RobotMap.DriveBase.FRONT_LEFT_MODULE_STEERING,
                                RobotMap.DriveBase.FRONT_LEFT_MODULE_ENCODER,
                                Constants.DriveBase.FRONT_LEFT_MODULE_STEER_OFFSET);

                frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
                                tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(2, 0),
                                Mk4SwerveModuleHelper.GearRatio.L2,
                                RobotMap.DriveBase.FRONT_RIGHT_MODULE_DRIVE,
                                RobotMap.DriveBase.FRONT_RIGHT_MODULE_STEERING,
                                RobotMap.DriveBase.FRONT_RIGHT_MODULE_ENCODER,
                                Constants.DriveBase.FRONT_RIGHT_MODULE_STEER_OFFSET);

                backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                                tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(4, 0),
                                Mk4SwerveModuleHelper.GearRatio.L2,
                                RobotMap.DriveBase.BACK_LEFT_MODULE_DRIVE,
                                RobotMap.DriveBase.BACK_LEFT_MODULE_STEERING,
                                RobotMap.DriveBase.BACK_LEFT_MODULE_ENCODER,
                                Constants.DriveBase.BACK_LEFT_MODULE_STEER_OFFSET);

                backRightModule = Mk4SwerveModuleHelper.createFalcon500(
                                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(6, 0),
                                Mk4SwerveModuleHelper.GearRatio.L2,
                                RobotMap.DriveBase.BACK_RIGHT_MODULE_DRIVE,
                                RobotMap.DriveBase.BACK_RIGHT_MODULE_STEERING,
                                RobotMap.DriveBase.BACK_RIGHT_MODULE_ENCODER,
                                Constants.DriveBase.BACK_RIGHT_MODULE_STEER_OFFSET);

                thetaController.enableContinuousInput(Math.PI, -Math.PI);
                pathController.setEnabled(true);

                tab.getLayout("Pigeon IMU", BuiltInLayouts.kList)
                                .withSize(2, 2)
                                .withPosition(8, 0)
                                .addNumber("Current Heading (Degree)", () -> getGyroscopeRotation().getDegrees());

                tab.getLayout("Pigeon IMU", BuiltInLayouts.kList)
                                .withSize(2, 2)
                                .withPosition(8, 0)
                                .addNumber("Current Heading (Radian)", () -> getGyroscopeRotation().getRadians());

                tab.getLayout("Odometry (Meters)", BuiltInLayouts.kList)
                                .withSize(2, 2)
                                .withPosition(8, 2)
                                .addNumber("X", () -> getPose().getX());

                tab.getLayout("Odometry (Meters)", BuiltInLayouts.kList)
                                .withSize(2, 2)
                                .withPosition(8, 2)
                                .addNumber("Y", () -> getPose().getY());

                tab.getLayout("Odometry (Meters)", BuiltInLayouts.kList)
                                .withSize(2, 2)
                                .withPosition(8, 2)
                                .addNumber("Theta", () -> getPose().getRotation().getDegrees());

        }

        /**
         * Gets the rotation of the Pigeon IMU. This rotation is relative to the
         * rotation the bot was in when the Pigeon IMU booted up.
         * 
         * @return The rotation of the Pigeon IMU as a Rotation2d
         */
        public Rotation2d getGyroscopeRotation() {
                return pigeon.getRotation2d();
        }

        /**
         * Sets current Pigeon IMU's rotation yaw value to a given value. All rotation
         * from then on will be relative to the given position.
         * 
         * @param newValue The new yaw value that the Pigeon IMU's rotation yaw value
         *                 should be set to.
         */
        public void setGyroscopeRotation(double newValue) {
                pigeon.setYaw(90);
        }

        /**
         * Sets the current Pigeon IMU's rotation yaw value to zero
         */
        public void zeroGyroscope() {
                setGyroscopeRotation(0);
        }

        /**
         * Moves the drivebase around by running the swerve modules.
         * 
         * @param vxMetersPerSecond     The desired meters per second the drivebase
         *                              should move in the x-axis direction
         * @param vyMetersPerSecond     The desired meters per second the drivebase
         *                              should move in the y-axis direction
         * @param omegaRadiansPerSecond The desired radians per second the robot should
         *                              rotate in a unit circle
         */
        public void drive(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond) {
                this.chassisSpeeds = new ChassisSpeeds(
                                vxMetersPerSecond,
                                vyMetersPerSecond,
                                omegaRadiansPerSecond);
        }

        /**
         * Moves the drivebase around by running the swerve modules.
         * 
         * @param chassisSpeeds The x, y, and theta the drivebase must move in.
         */
        public void drive(ChassisSpeeds chassisSpeeds) {
                this.chassisSpeeds = chassisSpeeds;
        }

        public void drive(Trajectory.State targetState, Rotation2d targetRotation) {
                ChassisSpeeds targetChassisSpeeds = pathController.calculate(
                                getPose(),
                                targetState,
                                targetRotation);
                drive(targetChassisSpeeds);
        }

        public void resetPathController() {
                xController.reset();
                yController.reset();
                thetaController.reset(
                                getGyroscopeRotation().getRadians(),
                                chassisSpeeds.omegaRadiansPerSecond);
        }

        /*
         * TODO: Test To See If Position Is Held When Pushed Around
         * TODO: Test To See If It Doesn't Interfere With Other Drive Functions
         */

        /**
         * Sets swerve modules into an x-shape so the drivebase cannot be pushed around.
         */
        public void lockDriveBase() {
                frontLeftModule.set(0, 45);
                frontRightModule.set(0, 135);
                backLeftModule.set(0, 135);
                backRightModule.set(0, 45);
        }

        /**
         * Resets the current odometry position to a given position. All odometry
         * tracking from then on will be relative to the given position.
         * 
         * @param pose The new pose that the odometry tracking should be set to. The
         *             translation aspect is a Translation2d meters and the rotation
         *             aspect is a Rotation2d.
         */
        public void resetOdometry(Pose2d pose) {
                m_odometry.resetPosition(pose, getGyroscopeRotation());
        }

        /**
         * Gets the current odometry reading of the drivebase.
         * 
         * @return A Pose2d with the current odometry position of the drivebase. The
         *         translation aspect is a Translation2d meters and the rotation
         *         aspect is a Rotation2d.
         */
        public Pose2d getPose() {
                double theta = 0;

                if (m_odometry.getPoseMeters().getRotation().getDegrees() < 0
                                && m_odometry.getPoseMeters().getRotation().getDegrees() > -180) {
                        theta = -m_odometry.getPoseMeters().getRotation().getDegrees();
                }

                else {
                        theta = 360 - m_odometry.getPoseMeters().getRotation().getDegrees();
                }

                return new Pose2d(
                                m_odometry.getPoseMeters().getX(),
                                m_odometry.getPoseMeters().getY(),
                                Rotation2d.fromDegrees(theta));
        }

        /**
         * Sets modules to desired states in both speed and direction.
         * 
         * @param desiredStates An array of desired states for the swerve module going
         *                      in order of front left, front right, back left, back
         *                      right.
         */
        public void setModuleStates(SwerveModuleState[] desiredStates) {
                SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,
                                Constants.DriveBase.MAX_VELOCITY_METERS_PER_SECOND);

                frontLeftModule.set(
                                desiredStates[0].speedMetersPerSecond
                                                / Constants.DriveBase.MAX_VELOCITY_METERS_PER_SECOND
                                                * Constants.DriveBase.MAX_VOLTAGE,
                                desiredStates[0].angle.getRadians());
                frontRightModule.set(
                                desiredStates[1].speedMetersPerSecond
                                                / Constants.DriveBase.MAX_VELOCITY_METERS_PER_SECOND
                                                * Constants.DriveBase.MAX_VOLTAGE,
                                desiredStates[1].angle.getRadians());
                backLeftModule.set(
                                desiredStates[2].speedMetersPerSecond
                                                / Constants.DriveBase.MAX_VELOCITY_METERS_PER_SECOND
                                                * Constants.DriveBase.MAX_VOLTAGE,
                                desiredStates[2].angle.getRadians());
                backRightModule.set(
                                desiredStates[3].speedMetersPerSecond
                                                / Constants.DriveBase.MAX_VELOCITY_METERS_PER_SECOND
                                                * Constants.DriveBase.MAX_VOLTAGE,
                                desiredStates[3].angle.getRadians());
        }

        @Override
        public void periodic() {
                SwerveModuleState[] states = Constants.DriveBase.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
                setModuleStates(states);

                m_odometry.update(
                                getGyroscopeRotation(),
                                states[0],
                                states[1],
                                states[2],
                                states[3]);
        }
}