// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OIConstants;
public class SwerveModule extends SubsystemBase{

    private CANSparkMax driveMotor;
    private CANSparkMax turningMotor;

    private AbsoluteEncoder absoluteEncoder;

    private PIDController turningPidControler;

    private final AnalogInput analogEncoder;
    private final boolean absoluteEncoderReversed;


    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, boolean absoluteEncoderReversed) {


        this.absoluteEncoderReversed = absoluteEncoderReversed;
        analogEncoder = new AnalogInput(absoluteEncoderId);

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);//Motor type suceptible to change
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);
       
        
        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

             absoluteEncoder = (AbsoluteEncoder) turningMotor.getEncoder();
        absoluteEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        absoluteEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);

        turningPidControler = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidControler.enableContinuousInput(-Math.PI, Math.PI);

     }


     public double getTurningPosition(){
        return absoluteEncoder.getPosition();
     }
     public double getTurningVelocity(){
        return absoluteEncoder.getVelocity();
     }
     public double getAbsoluteEncoderRad() {
      double angle = analogEncoder.getVoltage() / RobotController.getVoltage5V();
      angle *= 2.0 * Math.PI;
      return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
  }

  public SwerveModuleState getState() {
      return new SwerveModuleState(getTurningVelocity(), new Rotation2d(getTurningPosition()));
  }

  public void setDesiredState(SwerveModuleState state) {
      if (Math.abs(state.speedMetersPerSecond) < 0.001) {
          stop();
          return;
      }
      state = SwerveModuleState.optimize(state, getState().angle);
      turningMotor.set(turningPidControler.calculate(getTurningPosition(), state.angle.getRadians()));
      SmartDashboard.putString("Swerve[" + analogEncoder.getChannel() + "] state", state.toString());
  }


  public void stop() {
      driveMotor.set(0);
      turningMotor.set(0);
  }


}
