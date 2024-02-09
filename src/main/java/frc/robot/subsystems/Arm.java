package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;

public class Arm extends PIDSubsystem {

    private Spark armMotor;
    private AnalogEncoder armEncoder; 
    private final SimpleMotorFeedforward feedforward;

    public Arm() {
        super(new PIDController(0.1, 0.0, 0.0));
        getController().enableContinuousInput(-180, 180);

        armMotor = new Spark(Constants.armMotor);
        armEncoder = new AnalogEncoder(Constants.armEncoder);
        setSetpoint(90.0);
        feedforward = new SimpleMotorFeedforward(0.1, 0.05); 
    }

    public void armUp() { //amp
        double setpoint = 90.0;
        double feedforwardTerm = feedforward.calculate(setpoint);
        setSetpoint(setpoint + feedforwardTerm);
    }

    public void armDown() { //intake
        double setpoint = 0.0;
        double feedforwardTerm = feedforward.calculate(setpoint);
        setSetpoint(setpoint + feedforwardTerm);
    }

    public double getMeasurement() {
        return armEncoder.getDistance(); 
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'useOutput'");
    }

   

}

  

