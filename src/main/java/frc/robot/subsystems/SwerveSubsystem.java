// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Terrace Hunter

package frc.robot.subsystems;

// import com.kauailabs.navx.frc.AHRS;
// import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {

    ADIS16448_IMU gyro = new ADIS16448_IMU();

    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveMotorReversed,
            DriveConstants.kFrontLeftTurningMotorReversed,
            DriveConstants.kFrontLeftTurningAbsoluteEncoderPort,
            DriveConstants.kFrontLeftTurningAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveMotorReversed,
            DriveConstants.kFrontRightTurningMotorReversed,
            DriveConstants.kFrontRightTurningAbsoluteEncoderPort,
            DriveConstants.kFrontRightTurningAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveMotorReversed,
            DriveConstants.kBackLeftTurningMotorReversed,
            DriveConstants.kBackLeftTurningAbsoluteEncoderPort,
            DriveConstants.kBackLeftTurningAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveMotorReversed,
            DriveConstants.kBackRightTurningMotorReversed,
            DriveConstants.kBackRightTurningAbsoluteEncoderPort,
            DriveConstants.kBackRightTurningAbsoluteEncoderReversed);

    // private final AHRS gyro = new AHRS(SPI.Port.kMXP);
      public SwerveDriveKinematics kinematics = new SwerveDriveKinematics( 
        new Translation2d(DriveConstants.kWheelBase / 2, -DriveConstants.kTrackWidth / 2),
      new Translation2d(DriveConstants.kWheelBase / 2, DriveConstants.kTrackWidth / 2),
      new Translation2d(-DriveConstants.kWheelBase / 2, -DriveConstants.kTrackWidth / 2),
      new Translation2d(-DriveConstants.kWheelBase / 2, DriveConstants.kTrackWidth / 2));

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                 zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

     public void zeroHeading() {
        gyro.reset();
     }

     public double getHeading() {
          return Math.IEEEremainder(gyro.getAngle(), 360);
     }

     public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
     }

  

     

    @Override
    public void periodic() {
         SmartDashboard.putNumber("Robot Heading", getHeading());
         SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    private Pose2d getPose() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPose'");
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]); 
    }
    
}
