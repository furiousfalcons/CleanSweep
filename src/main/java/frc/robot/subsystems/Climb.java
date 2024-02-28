// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
  /** Creates a new Climb. */

  CANSparkMax climbMotor;

  public Climb() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    climbMotor = new CANSparkMax(12, MotorType.kBrushless);
  }

  public void climbUp() {
    climbMotor.set(.2);
  }

  public void climbStop() {
    climbMotor.set(0);
  }
}
