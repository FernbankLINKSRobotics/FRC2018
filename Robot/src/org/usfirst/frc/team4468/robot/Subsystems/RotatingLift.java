package org.usfirst.frc.team4468.robot.Subsystems;

import org.usfirst.frc.team4468.robot.Constants;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;

public class RotatingLift extends Subsystem {
	//// Declarations
	private VictorSP motor = new VictorSP(Constants.lifterPort1);
	
	private AnalogPotentiometer pot = new AnalogPotentiometer(
	        Constants.potPort, 
	        Constants.potRange, 
	        Constants.potOff
	);
	
	private DoubleSolenoid liftBreak = new DoubleSolenoid(Constants.BreakClampPort1, Constants.BreakClampPort2);
	
	
	//// Constructor and Command
	public RotatingLift() {}	
    public void initDefaultCommand() {}
    
    

    //// Motor Control
    /* Sets the motors to a specified speed
     */
    public void rotate(double speed) {
    		motor.set(speed);
    }
    
    /*  Stops motor movement
     */
    public void stop() {
    		motor.stopMotor();
    }
    
    
    //// Break Control
   /* Sets the state of the clamp that holds the cube
    */
   public void clamp(Value v) {
       liftBreak.set(v);
   }
   
   /* This returns the state of the solenoids
    * @return their similar state or an error is thrown
    */
   public Value getState() {
       return liftBreak.get();
   }
    
    
    
    //// Sensors
    /* The angle of the arm
     * @returns the angle in the form of a double
     */
    public double getAngle() {
        return pot.get() - Constants.angleOffset;
    }
}

