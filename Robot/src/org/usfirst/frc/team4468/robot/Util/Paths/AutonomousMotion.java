package org.usfirst.frc.team4468.robot.Util.Paths;

import org.usfirst.frc.team4468.robot.Util.Paths.MotionProfiler;
import org.usfirst.frc.team4468.robot.Commands.Drive.Move;

public class AutonomousMotion {
	private Move move;
	
	/**
     * Constructor for calling the 2d motion profiling
     * 
     * @param inc The increment, which is how far in advance you want to call the distance value
     * @param all of the parameters that are used in the 2d motion profiling constructor
     */
	public AutonomousMotion() {
		//increment can be accurate up to three decimal places
		//the increment is measured in distance
		move = new Move();
		
	}
	
	
	
	/**
     * Calling the motion profiling function in regard to current time
     * 
     * @param distance Current distance
     * @param time Current time
     * @return value Distance based on current time
     */
	public double executeTime(double time) {
		//This does not require the increment, but it does require distance
		move.getDistance(time);
	}

	
}
