// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// MAXSwerve 

package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends Command{

    // private final SwerveModule swerveSubsystem;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private SwerveSubsystem swerveSubsystem;
    private SwerveModule swerveModule;

    public SwerveJoystickCmd( SwerveSubsystem swereveSubsystem,
            Supplier<Boolean> fieldOrientedFunction) {
        
        this.swerveSubsystem = swereveSubsystem;
        addRequirements(swereveSubsystem, swerveModule);
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // 1. Get real-time joystick inputs
        double xSpeed = RobotContainer.xboxController.getRawAxis(0);
        double ySpeed =RobotContainer.xboxController.getRawAxis(1);
        double turningSpeed = RobotContainer.xboxController.getRawAxis(4);

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        // // 4. Construct desired chassis speeds
        //  ChassisSpeeds chassisSpeeds;
        // if (fieldOrientedFunction.get()) {
        //     // Relative to field
        //      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        //              xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        //  } else {
        //      // Relative to robot
        //     chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        //  }

        // // 5. Convert chassis speeds to individual module states
        //  SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
         swerveSubsystem.drive(xSpeed, ySpeed, turningSpeed, fieldOrientedFunction.get());
    
        
     }

     @Override
     public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

     @Override
     public boolean isFinished() {
         return false;
     }
}