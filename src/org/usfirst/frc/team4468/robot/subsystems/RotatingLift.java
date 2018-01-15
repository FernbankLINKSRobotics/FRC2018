package org.usfirst.frc.team4468.robot.subsystems;

import org.usfirst.frc.team4468.robot.Constants;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;

public class RotatingLift extends Subsystem {
	//// Declerations
	private SpeedControllerGroup motors = new SpeedControllerGroup(
			new VictorSP (Constants.lifterPort1),
			new VictorSP (Constants.lifterPort2)
	);
		
    public void initDefaultCommand() {}
    
    // Sets the motors to a specified speed
    public void Rotate(double speed) {
    		motors.set(speed);
    }
    
    // Stops motor movement
    public void end() {
    		motors.stopMotor();
    }
}

