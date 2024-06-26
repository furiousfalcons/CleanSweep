// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.InTakeShooter;


public class Shoot extends Command {
  /** Creates a new ClawGrab. */
  InTakeShooter inTakeShooter;
  public Shoot(InTakeShooter subsystem) {
    inTakeShooter = subsystem;
    addRequirements(subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }


  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    inTakeShooter.shoot();
    SmartDashboard.putBoolean("Is Shooting", inTakeShooter.isCurrentlyShooting());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    inTakeShooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return false;
  }
}
