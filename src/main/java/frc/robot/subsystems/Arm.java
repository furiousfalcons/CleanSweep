package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.signals.System_StateValue;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

    private CANSparkMax armMotorL, armMotorR;
    private DutyCycleEncoder armEncoder;

    private PIDController armController;
    private double setpoint = 0;
    long timeStart; //Duty cycle encoder is just a absoulte encoder (Search it up to learn more, it's fascinating!)
    /*Feed forward is not applicable here as brushless motors do not have a permant magnetic field 
    which is needed by the feed forward field (We have to find an alternatice method*/

    //The are has is contolled by two motors working togather
    public Arm() {    
        //The arm cann not fully rotate so a enabling continuous input is uncessary 
        armMotorL = new CANSparkMax(Constants.armMotorL, MotorType.kBrushless);
        armMotorR = new CANSparkMax(Constants.armMotorR, MotorType.kBrushless);

        armMotorL.restoreFactoryDefaults();
        armMotorR.restoreFactoryDefaults();

        armMotorL.setCANTimeout(250);
        armMotorR.setCANTimeout(250);

        armEncoder = new DutyCycleEncoder(Constants.armEncoder);
        armMotorL.setSmartCurrentLimit(40);
        armMotorR.setSmartCurrentLimit(40);

        armMotorL.follow(armMotorR);

        armMotorR.setInverted(true);

        armEncoder.setPositionOffset(.5);

        armMotorL.setCANTimeout(0);
        armMotorR.setCANTimeout(0);

        armMotorL.burnFlash();
        armMotorR.burnFlash();

        armController = new PIDController(0,0,0);
        armController.setTolerance(1);
      /*   if (System.currentTimeMillis() - timeStart < 500) {
            disable();
            armMotorR.set(.75);
            armMotorL.set( .75);
        } else if (System.currentTimeMillis() - timeStart > 500 && System.currentTimeMillis() - timeStart < 750) {
            enable();
            double distance = 150;
        setSetpoint(Math.max(distance, 135) );
        }*/

         //Increase the size/range of the set point
        

        
        
        
    }

    @Override
    public void periodic() {
        // double output = armController.calculate(getMeasurement(), setpoint);
        // armMotorR.setVoltage(output);
        // DriverStation.reportError(output + "", false);
        //armMotorR.setVoltage(-12);
    }



    public void setSetpoint(double setPoint) {
        this.setpoint = setPoint;
    }

    public double getMeasurement() {
        return  (armEncoder.getAbsolutePosition() - armEncoder.getPositionOffset()) * 360;
    }

    
  }
