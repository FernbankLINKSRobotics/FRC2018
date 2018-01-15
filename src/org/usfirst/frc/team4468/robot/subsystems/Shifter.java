package org.usfirst.frc.team4468.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team4468.robot.Constants;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/*
 * 
 */
public class Shifter extends Subsystem {
	
	 // Declares the 2 Solenoids used in shifting
	private DoubleSolenoid shift = new DoubleSolenoid(Constants.shifterPort1, Constants.shifterPort2);
	private boolean state;
	
	//// Constructior and Subystsem x
	public Shifter() {
		super();
		state = (shift.get() == Value.kForward);
	}
	
	@Override
	public void initDefaultCommand() {}
	
	
	
	//// Actuate
	 // If the shifter is shifted up the status is true. The robot is in high gear
	public void up() {
		shift.set(Value.kForward);
		state = true;
	}
	
	// If the shifter is not shifted up the status is false. The robot is not in high gear
	public void down() {
		shift.set(Value.kReverse);
		state = false;
	}
	
	
	
	//// Status
	/*  
	 * Returns the status of the shifter, as high gear being true or false @return
	 */
	public boolean isHighGear() {
		return state;
	}
	
	// Puts the status of the Solenoid in high gear as either true or fale on the Smart Dashbaard
	public void log() {
		// SmartDashboard.putBoolean("Gear high: ",state);
	}
}