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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class DriveBase extends SubsystemBase {

    private final WPI_Pigeon2 pigeon = new WPI_Pigeon2(RobotMap.DriveBaseHardware.PIGEON_IMU);

    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
            Constants.DriveBaseConstants.DRIVE_KINEMATICS,
            getGyroscopeRotation());

    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    private final PIDController xController = new PIDController(Constants.DriveBaseConstants.X_P_COEFF, 0, 0);
    private final PIDController yController = new PIDController(Constants.DriveBaseConstants.Y_P_COEFF, 0, 0);

    private final ProfiledPIDController thetaController = new ProfiledPIDController(
            Constants.DriveBaseConstants.THETA_P_COEFF, 0, 0,
            Constants.DriveBaseConstants.THETA_CONTROLLER_CONSTRAINTS);

    private final HolonomicDriveController pathController = new HolonomicDriveController(
            xController,
            yController,
            thetaController);

    private Rotation2d lastGivenRotation;

    public DriveBase() {
        //ShuffleboardTab tab = Shuffleboard.getTab("DriveBase");

        frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                Mk4SwerveModuleHelper.GearRatio.L2,
                RobotMap.DriveBaseHardware.FRONT_LEFT_MODULE_DRIVE,
                RobotMap.DriveBaseHardware.FRONT_LEFT_MODULE_STEERING,
                RobotMap.DriveBaseHardware.FRONT_LEFT_MODULE_ENCODER,
                Constants.DriveBaseConstants.FRONT_LEFT_MODULE_STEER_OFFSET);

        frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
                Mk4SwerveModuleHelper.GearRatio.L2,
                RobotMap.DriveBaseHardware.FRONT_RIGHT_MODULE_DRIVE,
                RobotMap.DriveBaseHardware.FRONT_RIGHT_MODULE_STEERING,
                RobotMap.DriveBaseHardware.FRONT_RIGHT_MODULE_ENCODER,
                Constants.DriveBaseConstants.FRONT_RIGHT_MODULE_STEER_OFFSET);

        backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                Mk4SwerveModuleHelper.GearRatio.L2,
                RobotMap.DriveBaseHardware.BACK_LEFT_MODULE_DRIVE,
                RobotMap.DriveBaseHardware.BACK_LEFT_MODULE_STEERING,
                RobotMap.DriveBaseHardware.BACK_LEFT_MODULE_ENCODER,
                Constants.DriveBaseConstants.BACK_LEFT_MODULE_STEER_OFFSET);

        backRightModule = Mk4SwerveModuleHelper.createFalcon500(
                Mk4SwerveModuleHelper.GearRatio.L2,
                RobotMap.DriveBaseHardware.BACK_RIGHT_MODULE_DRIVE,
                RobotMap.DriveBaseHardware.BACK_RIGHT_MODULE_STEERING,
                RobotMap.DriveBaseHardware.BACK_RIGHT_MODULE_ENCODER,
                Constants.DriveBaseConstants.BACK_RIGHT_MODULE_STEER_OFFSET);

        zeroGyroscope();

        thetaController.enableContinuousInput(Math.PI, -Math.PI);
        pathController.setEnabled(true);

    }

    /**
     * Gets the rotation of the Pigeon IMU. This rotation is relative to the
     * rotation the bot was in when the Pigeon IMU booted up.
     * 
     * @return The rotation of the Pigeon IMU as a Rotation2d
     */
    public Rotation2d getGyroscopeRotation() {
        double[] ypr = new double[3];
        pigeon.getYawPitchRoll(ypr);
        return Rotation2d.fromDegrees(ypr[0]);
    }

    /**
     * Sets current Pigeon IMU's rotation yaw value to a given value. All rotation
     * from then on will be relative to the given position.
     * 
     * @param newValue The new yaw value that the Pigeon IMU's rotation yaw value
     *                 should be set to.
     */
    public void setGyroscopeRotation(double newValue) {
        pigeon.setYaw(newValue);
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
        // determine ChassisSpeeds from path state and positional feedback control from
        // HolonomicDriveController
        lastGivenRotation = targetRotation;
        ChassisSpeeds targetChassisSpeeds = pathController.calculate(
                getPose(),
                targetState,
                targetRotation);
        // command robot to reach the target ChassisSpeeds
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

    public void resetOdometryWithLastHeading() {
        resetOdometry(new Pose2d(0, 0, lastGivenRotation));
    }

    /**
     * Gets the current odometry reading of the drivebase.
     * 
     * @return A Pose2d with the current odometry position of the drivebase. The
     *         translation aspect is a Translation2d meters and the rotation
     *         aspect is a Rotation2d.
     */
    public Pose2d getPose() {
        // double theta = 0;

        // if (m_odometry.getPoseMeters().getRotation().getDegrees() < 0
        // && m_odometry.getPoseMeters().getRotation().getDegrees() > -180) {
        // theta = -m_odometry.getPoseMeters().getRotation().getDegrees();
        // }

        // else {
        // theta = 360 - m_odometry.getPoseMeters().getRotation().getDegrees();
        // }

        // return new Pose2d(
        // m_odometry.getPoseMeters().getX(),
        // m_odometry.getPoseMeters().getY(),
        // Rotation2d.fromDegrees(theta));
        return m_odometry.getPoseMeters();
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
                Constants.DriveBaseConstants.MAX_VELOCITY_METERS_PER_SECOND);

        frontLeftModule.set(
                desiredStates[0].speedMetersPerSecond
                        / Constants.DriveBaseConstants.MAX_VELOCITY_METERS_PER_SECOND
                        * Constants.DriveBaseConstants.MAX_VOLTAGE,
                desiredStates[0].angle.getRadians());
        frontRightModule.set(
                desiredStates[1].speedMetersPerSecond
                        / Constants.DriveBaseConstants.MAX_VELOCITY_METERS_PER_SECOND
                        * Constants.DriveBaseConstants.MAX_VOLTAGE,
                desiredStates[1].angle.getRadians());
        backLeftModule.set(
                desiredStates[2].speedMetersPerSecond
                        / Constants.DriveBaseConstants.MAX_VELOCITY_METERS_PER_SECOND
                        * Constants.DriveBaseConstants.MAX_VOLTAGE,
                desiredStates[2].angle.getRadians());
        backRightModule.set(
                desiredStates[3].speedMetersPerSecond
                        / Constants.DriveBaseConstants.MAX_VELOCITY_METERS_PER_SECOND
                        * Constants.DriveBaseConstants.MAX_VOLTAGE,
                desiredStates[3].angle.getRadians());
    }

    @Override
    public void periodic() {
        SwerveModuleState[] states = Constants.DriveBaseConstants.DRIVE_KINEMATICS
                .toSwerveModuleStates(chassisSpeeds);
        setModuleStates(states);

        m_odometry.update(
                getGyroscopeRotation(),
                states[0],
                states[1],
                states[2],
                states[3]);
    }
}