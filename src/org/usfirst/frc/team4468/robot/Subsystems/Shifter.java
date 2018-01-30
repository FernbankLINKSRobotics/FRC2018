package org.usfirst.frc.team4468.robot.Subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team4468.robot.Constants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/*
 * 
 */
public class Shifter extends Subsystem {
	//// Declarations
	private DoubleSolenoid shift = new DoubleSolenoid(Constants.shifterPort1, Constants.shifterPort2);
	
	
	
	//// Constructor and Subsystem
	public Shifter() {
		super();
	}
	
	@Override
	public void initDefaultCommand() {}
	
	
	
	//// Actuate
	/* sets the shifter to a state
	 */
	public void set(Value v) {
	    shift.set(v);
	}
	
	
	
	//// Status
	/*  
	 * Returns the status of the shifter, as high gear being true or false @return
	 */
	public Value getState() {
	    return shift.get();
	}
}