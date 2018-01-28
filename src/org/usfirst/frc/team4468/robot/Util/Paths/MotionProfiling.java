package org.usfirst.frc.team4468.robot.Util.Paths;

public class MotionProfiling {
	
	double endDistance;
	double v_cruise;
	double a;
	double time;
	double cruiseRatio;
	double accelDistance;
	double[] x_values; 
	double[] y_values;
	
	public MotionProfiling(double[] xvalues, double[] yvalues, double cruiseV, double acceleration, double currentTime, double cruise_ratio, double accel_distance){
		endDistance = xvalues[xvalues.length];
		if (cruiseV > 6) {
			v_cruise = 6;
		}
		else {
			v_cruise = cruiseV;
		}
		a = acceleration;
		accelDistance = accel_distance;
		time = currentTime;
		cruiseRatio = cruise_ratio;
		x_values = xvalues;
		y_values = yvalues;
	}
	
	public double[] execute1D() {
	    	// The cruise distance is the total distance times the cruise ratio
	    	double returnVelocity;
	    	double returnAcceleration;
	    	double returnDistance;
	    	double accelDistance1 = (Math.pow(v_cruise, 2))/(2*a);
	    	if (accelDistance1 < (endDistance/2)*(1-(cruiseRatio/2))) {
	    		double timeTakenAD = Math.sqrt((2*accelDistance1)/a);
	    		double cruiseTime = (endDistance - 2*accelDistance1)/v_cruise;
	    		double totalTime = 2*timeTakenAD + cruiseTime;
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
	    			double t = time - (cruiseTime+timeTakenAD);
	    			returnDistance = accelDistance1 + cruiseTime*v_cruise + (v_cruise*t+((1/2)*(-a)*Math.pow(t, 2)));
	    			returnVelocity = v_cruise + (-a)*t;
	    			returnAcceleration = -a;
	    		}
	    	}
	    	else {
	    		double accelDistance2 = endDistance*((1-cruiseRatio)/2);
	    		double timeTakenAD = Math.sqrt((2*accelDistance2)/a);
	    		double cruiseTime = (endDistance*cruiseRatio)/v_cruise;
	    		double totalTime = timeTakenAD*2 + cruiseTime;
	    		if (time <= timeTakenAD) {
	    			returnDistance = (1/2)*a*Math.pow(time, 2);
	    			returnVelocity = a*time;
	    			returnAcceleration = a;
	    		}
	    		else if (time > timeTakenAD && time < (totalTime - timeTakenAD)) {
	    			double t = time - timeTakenAD;
	    			returnDistance = accelDistance2 + t*v_cruise;
	    			returnVelocity = v_cruise;
	    			returnAcceleration = 0;
	    		}
	    		else {
	    			double t = time - (cruiseTime+timeTakenAD);
	    			returnDistance = accelDistance2 + cruiseTime*v_cruise + (v_cruise*t+((1/2)*(-a)*Math.pow(t, 2)));
	    			returnVelocity = v_cruise + (-a)*t;
	    			returnAcceleration = -a;
	    		}
	    	}
	    	double[] array = {returnDistance, returnVelocity, returnAcceleration};
	    	return array;
	    }
	
	public double[] getMaxVelocity(double x_current, double y_current, double x_curve, double y_curve, double x_end, double y_end) {
		
		//x_values and y_values should only have three values: current position, end position, position in between current and end position
		double chord_slope = (y_end-y_current)/(x_end-x_current);
		double T_slope = -1/chord_slope;
		
		double x_inter = ((y_curve-T_slope*x_curve)-(y_current-chord_slope*x_current))/(chord_slope-T_slope);
		double y_inter = chord_slope*x_inter+(y_current-x_current*chord_slope);
		double chord_height = Math.hypot(x_curve-x_inter, y_curve-y_inter);
		double chord_length = Math.hypot(x_end-x_current, y_end-y_current);
		
		double radius = (chord_height/2)+(Math.pow(chord_length, 2)/(8*chord_height));
		double distance = 2*Math.asin(-chord_length/(2*radius))*radius;
		
		double[] returnArray = {Math.sqrt(radius*9.81), distance, chord_length};
		
		return returnArray;
	}
	
	public double[] execute2D() {
		
		int allValues = x_values.length;
		
		double[] acceleration = new double[x_values.length];
		double[] maxVelocities = new double[x_values.length];
		double[] distance = new double[x_values.length];
		double[] timeTaken = new double[x_values.length];
		
		
		
		//So that no more than 1 point is unaccounted for
		
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
    			stopIndex = i;
    		}
    		else {
    			j = j + 1;
    		}
    	}
    	for (int i = 0; i < (stopIndex+1); i++) {
    		//Sets the acceleration values
    		acceleration[i] = a;
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
    		acceleration[i] = -a;
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
	
	public double arraySum(double[] array, double index) {
		double sum = 0;
		for (int i = 0; i < index+1; i++) {
			sum = sum + array[i];
		}
		return sum;
	}
	 
}
