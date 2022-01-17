package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class DriveBase {
    private final PigeonIMU pigeon = new PigeonIMU(RobotMap.DriveBase.PIGEON_IMU);

    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    public DriveBase() {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        // TODO: Fix Swerve Module Configuration Based On Production Robot

        frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0),
                Mk4SwerveModuleHelper.GearRatio.L4,
                RobotMap.DriveBase.FRONT_LEFT_MODULE_DRIVE,
                RobotMap.DriveBase.FRONT_LEFT_MODULE_STEERING,
                RobotMap.DriveBase.FRONT_LEFT_MODULE_ENCODER,
                Constants.DriveBase.FRONT_LEFT_MODULE_STEER_OFFSET);

        frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0),
                Mk4SwerveModuleHelper.GearRatio.L4,
                RobotMap.DriveBase.FRONT_RIGHT_MODULE_DRIVE,
                RobotMap.DriveBase.FRONT_RIGHT_MODULE_STEERING,
                RobotMap.DriveBase.FRONT_RIGHT_MODULE_ENCODER,
                Constants.DriveBase.FRONT_RIGHT_MODULE_STEER_OFFSET);

        backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0),
                Mk4SwerveModuleHelper.GearRatio.L4,
                RobotMap.DriveBase.BACK_LEFT_MODULE_DRIVE,
                RobotMap.DriveBase.BACK_LEFT_MODULE_STEERING,
                RobotMap.DriveBase.BACK_LEFT_MODULE_ENCODER,
                Constants.DriveBase.BACK_LEFT_MODULE_STEER_OFFSET);

        backRightModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0),
                Mk4SwerveModuleHelper.GearRatio.L4,
                RobotMap.DriveBase.BACK_RIGHT_MODULE_DRIVE,
                RobotMap.DriveBase.BACK_RIGHT_MODULE_STEERING,
                RobotMap.DriveBase.BACK_RIGHT_MODULE_ENCODER,
                Constants.DriveBase.BACK_RIGHT_MODULE_STEER_OFFSET);
    }
}