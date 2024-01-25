// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
public class SwerveDrive extends SubsystemBase {
  /** Creates a new SwerveDrive. */
  private CANSparkMax frontLeft550, frontLeftNeo, frontRight550, frontRightNeo, 
                      backLeft550, backLeftNeo, backRight550, backRightNeo;
  
  

  private SwerveDriveKinematics swerve;

  public SwerveDrive() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
