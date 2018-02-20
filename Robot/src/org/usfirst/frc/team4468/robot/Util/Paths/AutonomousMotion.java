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
	
	//returning distance at current distance+the increment (how far in the future the distance is
	public double execute(double distance) {
		return motionProfile.getVelocity(distance, increment)[0];
	}

	
}
