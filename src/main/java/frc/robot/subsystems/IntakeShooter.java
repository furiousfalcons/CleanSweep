// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class InTakeShooter extends SubsystemBase {
     private NetworkTable tableInTakeShooter = NetworkTableInstance.getDefault().getTable("InTakeShooter");
    private CANSparkMax inTakeShooterMotor;
    private ColorSensorV3 colorSensor;
    private long time;
    private boolean isPickingUp;
    private boolean isShooting;
    private I2C.Port i2cPort;
    int proximity;

    //Note that the InTake and shooter mecanism is the same physical object
    public InTakeShooter() {
        i2cPort = I2C.Port.kOnboard;
        colorSensor = new ColorSensorV3(i2cPort);
        inTakeShooterMotor = new CANSparkMax(11, MotorType.kBrushless);
        time = System.currentTimeMillis(); // Last action time for shooter
        
    }

    @Override
    public void periodic() {
        proximity = colorSensor.getProximity();
     //   System.out.println(proximity);
    }

    public void inTake() {
       // if (proximity > 95 ) { // If object is inside, then stop
          //  isPickingUp = false;
           // inTakeShooterMotor.set(0.0);
           // tableInTakeShooter.getEntry("InTake").setBoolean(false);
     //   } else {
            isPickingUp = true;
            inTakeShooterMotor.set( Constants.inTakeMotorSpeed);
            tableInTakeShooter.getEntry("InTake").setBoolean(true); // Takes in ring
       // } 
        
    }

    public void shoot() {
        if (!isPickingUp) { // If object is not getting picked up, then continue
            isShooting = true;
            inTakeShooterMotor.set(-Constants.shootMotorSpeed);
            tableInTakeShooter.getEntry("Shoot").setBoolean(true);
            /* Removed timed section as having the shooting set to a hold button gives more control,
            for when unexpected issues arise */
            
        }
    }

    public void stop() { // Stops everything
        isShooting = false;
        isPickingUp = false;
        inTakeShooterMotor.set(0.0);
        tableInTakeShooter.getEntry("Shoot").setBoolean(false);
        tableInTakeShooter.getEntry("InTake").setBoolean(false);
    }
    //return if robot is currently picking up ring
    public boolean isTakingIn()
    {
        return isPickingUp;
    }
    //return if the robot is currently shooting a ring
    public boolean isCurrentlyShooting()
    {
        return isShooting;
    }
}

