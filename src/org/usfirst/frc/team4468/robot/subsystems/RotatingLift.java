package org.usfirst.frc.team4468.robot.subsystems;

import org.usfirst.frc.team4468.robot.Constants;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;

public class RotatingLift extends Subsystem {
	//// Declarations
	private SpeedControllerGroup motors = new SpeedControllerGroup(
	        new VictorSP (Constants.lifterPort1),
	        new VictorSP (Constants.lifterPort2)
	);
	
	private AnalogPotentiometer pot = new AnalogPotentiometer(
	        Constants.potPort, 
	        Constants.potRange, 
	        Constants.potOff
	);
	
	
	//// Constructor and Command
	public RotatingLift() {
		
	}	
	
    public void initDefaultCommand() {}
    
    

    //// Motor Control
    /* Sets the motors to a specified speed
     */
    public void rotate(double speed) {
    		motors.set(speed);
    }
    /*  Stops motor movement
     */
    public void stop() {
    		motors.stopMotor();
    }
    
    
    //// Sensors
    /* The angle of the arm
     * @returns the angle in the form of a double
     */
    public double getAngle() {
        return pot.get();
    }
}

