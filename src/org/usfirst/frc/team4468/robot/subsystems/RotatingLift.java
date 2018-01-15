package org.usfirst.frc.team4468.robot.subsystems;

import org.usfirst.frc.team4468.robot.Constants;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;

public class RotatingLift extends Subsystem {
	//// Declerations
	//private SpeedControllerGroup motors = new SpeedControllerGroup(
	private VictorSP left = new VictorSP (Constants.lifterPort1);
	private VictorSP right = new VictorSP (Constants.lifterPort1);
	
	private double s;
	//);
	
	public RotatingLift(double d) {
		s = d;
	}
	
    public void initDefaultCommand() {}
    
    // Sets the motors to a specified speed
    public void rotate(double speed) {
    		left.set(speed);
    		right.set(speed);
    }
    
    // Stops motor movement
    public void end() {
    		left.stopMotor();
    		right.stopMotor();
    }
}

