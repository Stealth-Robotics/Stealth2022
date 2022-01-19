package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class DriveBase extends SubsystemBase {

    private final PigeonIMU pigeon = new PigeonIMU(RobotMap.DriveBase.PIGEON_IMU);

    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    public DriveBase() {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

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
    }

    public Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(pigeon.getFusedHeading());
    }

    public void setGyroscopeRotation(double newValue) {
        pigeon.setFusedHeading(newValue);
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

    @Override
    public void periodic() {
        SwerveModuleState[] states = Constants.DriveBase.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.DriveBase.MAX_VELOCITY_METERS_PER_SECOND);

        frontLeftModule.set(
                states[0].speedMetersPerSecond / Constants.DriveBase.MAX_VELOCITY_METERS_PER_SECOND
                        * Constants.DriveBase.MAX_VOLTAGE,
                states[0].angle.getRadians());
        frontRightModule.set(
                states[1].speedMetersPerSecond / Constants.DriveBase.MAX_VELOCITY_METERS_PER_SECOND
                        * Constants.DriveBase.MAX_VOLTAGE,
                states[1].angle.getRadians());
        backLeftModule.set(
                states[2].speedMetersPerSecond / Constants.DriveBase.MAX_VELOCITY_METERS_PER_SECOND
                        * Constants.DriveBase.MAX_VOLTAGE,
                states[2].angle.getRadians());
        backRightModule.set(
                states[3].speedMetersPerSecond / Constants.DriveBase.MAX_VELOCITY_METERS_PER_SECOND
                        * Constants.DriveBase.MAX_VOLTAGE,
                states[3].angle.getRadians());
    }
}