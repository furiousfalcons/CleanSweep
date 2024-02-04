package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix6.signals.ControlModeValue;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants;

public class IntakeShooter {
    private NetworkTable tableInTakeShooter = NetworkTableInstance.getDefault().getTable("InTakeShooter");
    private Spark inTakeShooterMotor;
    private DigitalInput colorSensor;
    private long time;
    private boolean isPickingUp;
    private boolean isShooting;
    private Arm arm;

    public IntakeShooter() {
        colorSensor = new DigitalInput(1);
        inTakeShooterMotor = new Spark(Constants.InTakeShooterMotor);
        time = System.currentTimeMillis(); // Last action time for shooter
        arm = new Arm(); // Initialize the Arm
    }

    public void inTake() {
        if (!colorSensor.get() || isShooting) { // If object is inside or shooting, then stop
            isPickingUp = false;
            inTakeShooterMotor.set(0.0);
            tableInTakeShooter.getEntry("InTake").setBoolean(false);
        } else {
            isPickingUp = true;
            inTakeShooterMotor.set( Constants.inTakeMotorSpeed);
            tableInTakeShooter.getEntry("InTake").setBoolean(true); // Takes in ring
        }
    }

    public void shoot() {
        if (!isPickingUp) { // If object is not getting picked up, then continue
            isShooting = true;
            inTakeShooterMotor.set(Constants.shootMotorSpeed);
            tableInTakeShooter.getEntry("Shoot").setBoolean(true);
            if (System.currentTimeMillis() - time > Constants.shooterTime) { // Stop if certain time is elapsed
                stop();
                arm.armDown();
            }
        }
    }

    public void stop() { // Stops everything
        isShooting = false;
        isPickingUp = false;
        inTakeShooterMotor.set(0.0);
        tableInTakeShooter.getEntry("Shoot").setBoolean(false);
        tableInTakeShooter.getEntry("InTake").setBoolean(false);
    }
}
