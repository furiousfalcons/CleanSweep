package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;

public class Arm extends PIDSubsystem {

    private CANSparkMax armMotorL, armMotorR;
    private DutyCycleEncoder armEncoder; //Duty cycle encoder is just a absoulte encoder (Search it up to learn more, it's fascinating!)
    /*Feed forward is not applicable here as brushless motors do not have a permant magnetic field 
    which is needed by the feed forward field (We have to find an alternatice method*/

    //The are has is contolled by two motors working togather
    public Arm() {
        super(new PIDController(.011, 0.000, 0.000));
        enable();
        
        //The arm cann not fully rotate so a enabling continuous input is uncessary 
        armMotorL = new CANSparkMax(Constants.armMotorL, MotorType.kBrushless);
        armMotorR = new CANSparkMax(Constants.armMotorR, MotorType.kBrushless);
        armEncoder = new DutyCycleEncoder(Constants.armEncoder);
        setSetpoint(getMeasurement());
        armMotorL.restoreFactoryDefaults();
        armMotorR.restoreFactoryDefaults();
        getController().setTolerance(10);

        armMotorL.setCANTimeout(250);
        armMotorR.setCANTimeout(250);

      
        armMotorL.setSmartCurrentLimit(40);
        armMotorR.setSmartCurrentLimit(40);
        

        armMotorL.setCANTimeout(250);
        armMotorR.setCANTimeout(250);

        armMotorL.burnFlash();
        armMotorR.burnFlash();

        
    
         //Increase the size/range of the set point
        

        
        
        
    }



    public void armUp() { //amp
        double distance = getSetpoint() - 2;
        setSetpoint(Math.max(distance, 135) );

    }

    public void armDown() { //intake
        double distance = getSetpoint() + 2;
        
        setSetpoint(Math.min(distance,270));
        
    }

    public double getMeasurement() {
        return  armEncoder.getAbsolutePosition() * 360;
    }

    @Override
    protected void useOutput(double output, double setpoint) {
       // armMotorR.set( -output);
       // armMotorL.set(-output);
      // System.out.println(armEncoder.getAbsolutePosition() * 360);
       // System.out.println(setpoint);
       System.out.println(output);
       // System.out.println(setpoint - (armEncoder.getAbsolutePosition() * 360));
    }
}
    
