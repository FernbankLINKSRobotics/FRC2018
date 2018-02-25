package org.usfirst.frc.team4468.robot.Util.Paths;

import org.usfirst.frc.team4468.robot.Util.Paths.MotionProfiler;

public class AutonomousMotion {
	static double increment;
	static double[] x_values;
	static double[] y_values;
	static double accel;
	static double max_velocity;
	static double limit;
	static double max_acceleration;
	
	/**
     * Constructor for calling the 2d motion profiling
     * 
     * @param inc The increment, which is how far in advance you want to call the distance value
     * @param all of the parameters that are used in the 2d motion profiling constructor
     */
	public AutonomousMotion(double inc, double[] xvalues, double[] yvalues, double acceleration, double max, double limitingFactor, double maxAcceleration) {
		//increment can be accurate up to three decimal places
		//the increment is measured in distance
		increment = inc;
		x_values = xvalues;
		y_values = yvalues;
		accel = acceleration;
		max_velocity = max;
		limit = limitingFactor;
		max_acceleration = maxAcceleration;
		
	}
	
	private static MotionProfiler motionProfile = new MotionProfiler(x_values, y_values, accel, max_velocity, limit, max_acceleration);
	
	/**
     * Calling the motion profiling function in regard to current distance
     * 
     * @param distance Current distance
     * @return value Distance at current distance+the increment
     */
	public double executeDistance(double distance) {
		double value;
		double check = motionProfile.getVelocity(distance, increment)[0]-distance;
		if (check<0.0) {
			value = 0.0;
		}
		else if ((check+distance)>motionProfile.getVelocity(0.0, 0.0)[3]) {
			value = 0.0;
		}
		else {
			value = check;
		}
		return value;
	}
	
	/**
     * Calling the motion profiling function in regard to current time
     * 
     * @param distance Current distance
     * @param time Current time
     * @return value Distance based on current time
     */
	public double executeTime(double time, double distance) {
		//This does not require the increment, but it does require distance
		double check = motionProfile.execute2D(time, 0.0, 0.0)[0]-distance;
		double value;
		if (check<0.0) {
			value = 0.0;
		}
		else if ((check+distance)>motionProfile.getVelocity(0.0, 0.0)[3]) {
			value = 0.0;
		}
		else {
			value = check;
		}
		return value;
	}

	
}
