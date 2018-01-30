package org.usfirst.frc.team4468.robot.Util.Paths;

public class MotionProfiler {
	
	double endDistance;
	double v_cruise;
	double a;
	double cruiseRatio;
	double accelDistance;
	double[] x_values; 
	double[] y_values;
	
	/**
     * Constructor. Creates a new MotionProfiling instance.
     * 
     * @param cruiseV The cruise velocity
     * @param acceleration The starting acceleration and ending deceleration
     * @param currentTime The current time
     * @param cruise_ratio The ratio setting MINIMUM distance needed to be at a constant velocity
     * @param target The 1d end distance
     */
	public MotionProfiler(double cruiseV, double acceleration, double cruise_ratio, double target){
		endDistance = target;
		if (cruiseV > 6) {
			v_cruise = 6;
		}
		else {
			v_cruise = cruiseV;
		}
		a = acceleration;
		cruiseRatio = cruise_ratio;
	}
	
	/**
     * Constructor. Creates a new MotionProfiling instance.
     * 
     * @param xvalues An array of all the x values the robot will cross
     * @param yvalues An array of all the y values the robot will cross
     * @param acceleration The starting acceleration and ending deceleration
     * @param currentTime The current time
     * @param accel_distance The distance allotted for the robot to accelerate and decelerate
     */
	public MotionProfiler(double[] xvalues, double[] yvalues, double accel_distance) {
		accelDistance = accel_distance;
		x_values = xvalues;
		y_values = yvalues;
	}
	
	/**
     * 1d motion profiling
     * 
     * @return An array of expected distance, velocity, and acceleration based on current time
     */
	public double[] execute1D(double time) {
	    	// The cruise ratio is the MINIMUM distance needed to be at cruise velocity
	    	double returnVelocity;
	    	double returnAcceleration;
	    	double returnDistance;
	    	//calculating acceleration distance considering what portion of the distance cruise velocity will be at
	    	double accelDistance1 = (Math.pow(v_cruise, 2))/(2*a);
	    	if (accelDistance1 < (endDistance/2)*(1-(cruiseRatio/2))) {
	    		//below is only initiated if the acceleration distance allows for allotted cruise distance
	    		//calculating time taken for each segment
	    		double timeTakenAD = Math.sqrt((2*accelDistance1)/a);
	    		double cruiseTime = (endDistance - 2*accelDistance1)/v_cruise;
	    		double totalTime = 2*timeTakenAD + cruiseTime;
	    		
	    		//returning values based on current time
	    		if (time <= timeTakenAD) {
	    			returnDistance = (1/2)*a*Math.pow(time, 2);
	    			returnVelocity = a*time;
	    			returnAcceleration = a;
	    		}
	    		else if (time > timeTakenAD && time < (totalTime - timeTakenAD)) {
	    			double t = time - timeTakenAD;
	    			returnDistance = accelDistance1 + t*v_cruise;
	    			returnVelocity = v_cruise;
	    			returnAcceleration = 0;
	    		}
	    		else {
	    			//the same properties that go for the acceleration period also go for the deceleration period
	    			double t = time - (cruiseTime+timeTakenAD);
	    			returnDistance = accelDistance1 + cruiseTime*v_cruise + (v_cruise*t+((1/2)*(-a)*Math.pow(t, 2)));
	    			returnVelocity = v_cruise + (-a)*t;
	    			returnAcceleration = -a;
	    		}
	    	}
	    	else {
	    		//lowers accel distance to fit the cruise ratio
	    		double accelDistance2 = endDistance*((1-cruiseRatio)/2);
	    		//setting a lower cruise velocity to account for the lower accel distance
	    		double v_cruise1 = Math.sqrt(2*a*accelDistance2);
	    		double timeTakenAD = Math.sqrt((2*accelDistance2)/a);
	    		double cruiseTime = (endDistance*cruiseRatio)/v_cruise1;
	    		double totalTime = timeTakenAD*2 + cruiseTime;
	    		//Using the same calculations as before except with v_cruise1 instead of v_cruise
	    		if (time <= timeTakenAD) {
	    			returnDistance = (1/2)*a*Math.pow(time, 2);
	    			returnVelocity = a*time;
	    			returnAcceleration = a;
	    		}
	    		else if (time > timeTakenAD && time < (totalTime - timeTakenAD)) {
	    			double t = time - timeTakenAD;
	    			returnDistance = accelDistance2 + t*v_cruise1;
	    			returnVelocity = v_cruise1;
	    			returnAcceleration = 0;
	    		}
	    		else {
	    			double t = time - (cruiseTime+timeTakenAD);
	    			returnDistance = accelDistance2 + cruiseTime*v_cruise1 + (v_cruise1*t+((1/2)*(-a)*Math.pow(t, 2)));
	    			returnVelocity = v_cruise1 + (-a)*t;
	    			returnAcceleration = -a;
	    		}
	    	}
	    	
	    	//returning the current distance, velocity, and acceleration
	    	double[] array = {returnDistance, returnVelocity, returnAcceleration};
	    	return array;
	    }
	
	/**
     * Calculates values depending on the curvature of a three-point arc
     * 
     * @return An array of maximum velocity depending on curvature, distance covered (arc length), and the length of the chord between the arc
     */
	public double[] getMaxVelocity(double x_current, double y_current, double x_curve, double y_curve, double x_end, double y_end) {
		//These three inputs will come together to create an arc: an estimation of what the robot's motion will be
		//This creates a chord between the first and last point on the arc
		double chord_slope = (y_end-y_current)/(x_end-x_current);
		//This creates a slope of the line perpendicular to the equation of the chord
		double T_slope = -1/chord_slope;
		//Finding the intersection between the chord and the perpendicular line, which used x_curve and y_curve to be created
		double x_inter = ((y_curve-T_slope*x_curve)-(y_current-chord_slope*x_current))/(chord_slope-T_slope);
		double y_inter = chord_slope*x_inter+(y_current-x_current*chord_slope);
		//finding the distance between the chord and perpendicular line intersection point and the x_curve and y_curve point
		double chord_height = Math.hypot(x_curve-x_inter, y_curve-y_inter);
		//finding the length of the chord
		double chord_length = Math.hypot(x_end-x_current, y_end-y_current);
		
		//finding the radius and arc length based on chord_height and chord_length
		double radius = (chord_height/2)+(Math.pow(chord_length, 2)/(8*chord_height));
		double distance = 2*Math.asin(-chord_length/(2*radius))*radius;
		//calculating maximum velocity considering curvature of the arc
		double maxVelocity = Math.sqrt(radius*9.81);
		
		double[] returnArray = {maxVelocity, distance, chord_length};
		return returnArray;
	}
	
	/**
     * 2d motion profiling
     * 
     * @return An array of expected distance, velocity, and acceleration based on current time
     */
	public double[] execute2D(double time) {
		
		int allValues = x_values.length;
		
		double[] acceleration = new double[x_values.length];
		double[] maxVelocities = new double[x_values.length];
		double[] distance = new double[x_values.length];
		double[] timeTaken = new double[x_values.length];

		for (int i = 0; i < allValues; i = i + 2) {
				maxVelocities[i] = getMaxVelocity(x_values[i], y_values[i], x_values[i+1], y_values[i+1], x_values[i+2], y_values[i+2])[0];
				//Sets the max velocity according to the next two points
				if (i < allValues) {
					maxVelocities[i+1] = getMaxVelocity(x_values[i], y_values[i], x_values[i+1], y_values[i+1], x_values[i+2], y_values[i+2])[0];
					//Shows distance that WAS accomplished, why we add +1 to i
					distance[i+1] = (getMaxVelocity(x_values[i], y_values[i], x_values[i+1], y_values[i+1], x_values[i+2], y_values[i+2])[1])/2 + arraySum(distance, i);
					if (i < (allValues-1)) {
						distance[i+2] = (getMaxVelocity(x_values[i], y_values[i], x_values[i+1], y_values[i+1], x_values[i+2], y_values[i+2])[1])/2 + arraySum(distance, i+1);
					}
					//Use if statements so we don't get an error about array size
				}
		}
		if (x_values.length % 2 == 0) {
			maxVelocities[allValues] = getMaxVelocity(x_values[allValues-2], y_values[allValues-2], x_values[allValues-1], y_values[allValues-1], x_values[allValues], y_values[allValues])[0];
		}
		//if the amount of values is even, set the last value manually
		
		//Sets acceleration depending on if two velocities are different
		//Acceleration period will most likely happen once every three points
		for (int i = 1; i < x_values.length; i = i+2) {
			if (maxVelocities[i-1] != maxVelocities[i]) {
				double tdistance = distance[i+1]-distance[i];
				acceleration[i] = (Math.pow(maxVelocities[i+1], 2)-Math.pow(maxVelocities[i], 2))/(2*tdistance);
				//acceleration is set for the current to the next point
			}
		}
		
		distance[0] = 0;
		maxVelocities[0] = 0;
		//the start distance and max velocity will be zero
	
		//The velocities set below will override those previously set in the maxVelocities array based on acceleration, effectively clamping them
		
		//SETTING ACCELERATION
    	int stopIndex = 0;
    	int j = 0;
    	for (int i = 0; i < x_values.length; i++) {
    		if (distance[j]>accelDistance) {
    			//finds the distance array indexes where the starting acceleration applies
    			stopIndex = j;
    		}
    		else {
    			j = j + 1;
    		}
    	}
    	//Setting acceleration based on accel distance specified
    	double tdaccel = Math.pow(maxVelocities[stopIndex], 2)/(2*distance[stopIndex]);
    	for (int i = 0; i < (stopIndex+1); i++) {
    		//Sets the acceleration values
    		acceleration[i] = tdaccel;
    	}
    	//this loop utilizes the first max velocity being zero to set the next max velocity
    	for (int i = 1; i < (stopIndex+1); i++) {
    		//Sets the current velocity during the start acceleration period
    		maxVelocities[i] = Math.sqrt(Math.pow(maxVelocities[i-1], 2)+2*acceleration[i]*(distance[i]-distance[i-1]));
    	}
    	
    	//SETTING DECELERATION 
    	int endIndex = 0;
    	int h = 0;
    	for (int i = 0; i < x_values.length; i++) {
    		//finds the indexes where the deceleration period applies by subtracting total from current distance
    		if ((distance[allValues]-distance[h])>accelDistance) {
    			h = h + 1;
    		}
    		else {
    			endIndex = i;
    		}
    	}
    	for (int i = endIndex; i < (allValues+1); i++) {
    		//Sets the deceleration values
    		acceleration[i] = -tdaccel;
    	}
    	//Changes the current velocity during the deceleration period
    	for (int i = endIndex; i < (allValues+1); i++) {
    		maxVelocities[i] = Math.sqrt(Math.pow(maxVelocities[i-1], 2)+2*acceleration[i]*(distance[i]-distance[i-1]));
    	}
    	
    	
    	
    	//Setting time
    	for (int i = 1; i < x_values.length; i++) {
    		//Change in distance
    		double temp = distance[i+1] - distance[i];
    		if (acceleration[i-1] != 0) {
    			//the timeTaken array measures time COMPLETED whereas the velocity and acceleration array outputs FUTURE values (for the next increment)
    			//This is why we subtract i by 1 for velocity and acceleration
    			timeTaken[i] = (maxVelocities[i]-maxVelocities[i-1])/acceleration[i-1];
    		}
    		else {
    			//a reformat of d = v*t; there is no acceleration
    			timeTaken[i] = temp/maxVelocities[i-1];
    		}
    	}
    	
    	//establishing return values
    	double returnDistance;
    	double returnVelocity;
    	double returnAccel;
    	
		int currentIndex = 0;
		for (int i = 0; i < allValues; i++) {
			int l = 0;
			//finding where in the path the robot currently is based on current time
			//This is where the array function below is used
			if (time > arraySum(timeTaken, l)) {
				currentIndex = i;
			}
			else {
				l = l+1;
			}
		}
		
		//calculating the different in time and distance (between current position and last increment)
		double timeDifference = time - arraySum(timeTaken, currentIndex);
		double distanceChange = maxVelocities[currentIndex]*timeDifference+(.5)*acceleration[currentIndex]*Math.pow(timeDifference, 2);
		//calculating total distance covered
		returnDistance = distance[currentIndex] + distanceChange;
		
		if (acceleration[currentIndex]!=0) {
			returnAccel = acceleration[currentIndex];
			//calculating predicted velocity that the robot is currently at based on change in distance covered
			returnVelocity = Math.sqrt(2*returnAccel*distanceChange+Math.pow(maxVelocities[currentIndex], 2));
		}
		else {
			returnAccel = 0;
			//velocity should stay the same if acceleration is 0
			returnVelocity = maxVelocities[currentIndex];
		}
		
		//stores what the distance covered, current velocity, and current acceleration values should be in an array
		double[] returnArray = {returnDistance, returnVelocity, returnAccel};
		return returnArray;
	}
	
	/**
     * Calculates the sum of an array up to a certain index
     * 
     * @param array The array to be added up
     * @param index The index that the sum goes up to
     * 
     * @return The sum
     */
	public double arraySum(double[] array, double index) {
		double sum = 0;
		for (int i = 0; i < index+1; i++) {
			sum = sum + array[i];
		}
		return sum;
	}
	 
}

