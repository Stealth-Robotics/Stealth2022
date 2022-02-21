package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

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
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
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

        ProfiledPIDController thetaController;

        private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

        public DriveBase() {
                ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

                //pigeon.configFactoryDefault();
                zeroGyroscope();

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
                                                .withPosition(0, 0),
                                Mk4SwerveModuleHelper.GearRatio.L2,
                                RobotMap.DriveBase.FRONT_RIGHT_MODULE_DRIVE,
                                RobotMap.DriveBase.FRONT_RIGHT_MODULE_STEERING,
                                RobotMap.DriveBase.FRONT_RIGHT_MODULE_ENCODER,
                                Constants.DriveBase.FRONT_RIGHT_MODULE_STEER_OFFSET);

                backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                                tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(0, 0),
                                Mk4SwerveModuleHelper.GearRatio.L2,
                                RobotMap.DriveBase.BACK_LEFT_MODULE_DRIVE,
                                RobotMap.DriveBase.BACK_LEFT_MODULE_STEERING,
                                RobotMap.DriveBase.BACK_LEFT_MODULE_ENCODER,
                                Constants.DriveBase.BACK_LEFT_MODULE_STEER_OFFSET);

                backRightModule = Mk4SwerveModuleHelper.createFalcon500(
                                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(0, 0),
                                Mk4SwerveModuleHelper.GearRatio.L2,
                                RobotMap.DriveBase.BACK_RIGHT_MODULE_DRIVE,
                                RobotMap.DriveBase.BACK_RIGHT_MODULE_STEERING,
                                RobotMap.DriveBase.BACK_RIGHT_MODULE_ENCODER,
                                Constants.DriveBase.BACK_RIGHT_MODULE_STEER_OFFSET);

                thetaController = new ProfiledPIDController(
                                Constants.DriveBase.THETA_P_CONTROLLER,
                                Constants.DriveBase.THETA_I_CONTROLLER,
                                Constants.DriveBase.THETA_D_CONTROLLER,
                                Constants.DriveBase.THETA_CONTROLLER_CONSTRAINTS);

                thetaController.enableContinuousInput(-Math.PI, Math.PI);
        }

        public Rotation2d getGyroscopeRotation() {
                return pigeon.getRotation2d();
        }

        public void setGyroscopeRotation(double newValue) {
                pigeon.setYaw(-90);
        }

        public void zeroGyroscope() {
                setGyroscopeRotation(0);
        }

        public void drive(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond) {
                this.chassisSpeeds = new ChassisSpeeds(
                                vxMetersPerSecond,
                                vyMetersPerSecond,
                                omegaRadiansPerSecond);
        }

        public void drive(ChassisSpeeds chassisSpeeds) {
                this.chassisSpeeds = chassisSpeeds;
        }

        /*
         * TODO: Test To See If Position Is Held When Pushed Around
         * TODO: Test To See If It Doesn't Interfere With Other Drive Functions
         */
        public void lockDriveBase() {
                frontLeftModule.set(0, 45);
                frontRightModule.set(0, 135);
                backLeftModule.set(0, 135);
                backRightModule.set(0, 45);
        }

        public void resetOdometry(Pose2d pose) {
                m_odometry.resetPosition(pose, getGyroscopeRotation());
        }

        public Pose2d getPose() {
                return m_odometry.getPoseMeters();
        }

        public SwerveControllerCommand getSwerveControllerCommand(Trajectory trajectory) {
                return new SwerveControllerCommand(
                                trajectory,
                                () -> getPose(),
                                Constants.DriveBase.DRIVE_KINEMATICS,
                                new PIDController(Constants.DriveBase.X_P_CONTROLLER,
                                                Constants.DriveBase.X_I_CONTROLLER,
                                                Constants.DriveBase.X_D_CONTROLLER),
                                new PIDController(Constants.DriveBase.Y_P_CONTROLLER,
                                                Constants.DriveBase.Y_I_CONTROLLER,
                                                Constants.DriveBase.Y_D_CONTROLLER),
                                thetaController,
                                this::setModuleStates,
                                this);
        }

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

                System.out.println(getGyroscopeRotation());
        }
}