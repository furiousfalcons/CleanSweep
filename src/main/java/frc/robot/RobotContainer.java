// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

 package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

  import java.util.List;

 import edu.wpi.first.math.controller.PIDController;
 import edu.wpi.first.math.controller.ProfiledPIDController;
 import edu.wpi.first.math.geometry.Pose2d;
 import edu.wpi.first.math.geometry.Rotation2d;
 import edu.wpi.first.math.geometry.Translation2d;
 import edu.wpi.first.math.trajectory.TrajectoryConfig;
 import edu.wpi.first.math.trajectory.TrajectoryGenerator;
 import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
 import edu.wpi.first.wpilibj2.command.InstantCommand;
 import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
 import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
 import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
 import frc.robot.Constants.DriveConstants;
 import frc.robot.Constants.OIConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;


 /**
  * This class is where the bulk of the robot should be declared. Since Command-based is a
  * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
  * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
  * subsystems, commands, and trigger mappings) should be declared here.
  */
 public class RobotContainer {

     private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
     
      public final static XboxController xboxController = new XboxController(0);
     
  
     private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);

     public RobotContainer() {
      swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                 swerveSubsystem,
                 () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));
        
     }

    private void configureButtonBindings() {
    }

    public Command getAutonomousCommand() {
      return null;

        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
//         // 1. Create trajectory settings
//         TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
//                 AutoConstants.kMaxSpeedMetersPerSecond,
//                 AutoConstants.kMaxAccelerationMetersPerSecondSquared)
//                         .setKinematics(DriveConstants.kDriveKinematics);

//         // 2. Generate trajectory
//         Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
//                 new Pose2d(0, 0, new Rotation2d(0)),
//                 List.of(
//                         new Translation2d(1, 0),
//                         new Translation2d(1, -1)),
//                 new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
//                 trajectoryConfig);

//         // 3. Define PID controllers for tracking trajectory
//         PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
//         PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
//         ProfiledPIDController thetaController = new ProfiledPIDController(
//                 AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
//         thetaController.enableContinuousInput(-Math.PI, Math.PI);

//         // 4. Construct command to follow trajectory
//         SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
//                 trajectory,
//                 swerveSubsystem::getPose,
//                 DriveConstants.kDriveKinematics,
//                 xController,
//                 yController,
//                 thetaController,
//                 swerveSubsystem::setModuleStates,
//                 swerveSubsystem);

//         // 5. Add some init and wrap-up, and return everything
//         return new SequentialCommandGroup(
//                 new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
//                 swerveControllerCommand,
//                 new InstantCommand(() -> swerveSubsystem.stopModules()));
 }
}