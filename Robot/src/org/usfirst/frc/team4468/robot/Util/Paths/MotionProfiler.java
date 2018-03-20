package org.usfirst.frc.team4468.robot.Util.Paths;

public class MotionProfiler {
	double endDistance;
	double v_cruise;
	double a;
	double ta;
	double cruiseRatio;
	double limit;
	double accelDistance;
	double max_acceleration;
	double max_velocity;
	double[] x_values; 
	double[] y_values;
	boolean negative;
	
	boolean left_one;
	boolean back_one;
	boolean left_two;
	boolean back_two;
	
	double robot_width = .1;
	
	
	/**
     * Constructor. Creates a new MotionProfiling instance.
     * 
     * @param cruiseV The cruise velocity
     * @param maxV The maximum possible velocity
     * @param acceleration The starting acceleration and ending deceleration
     * @param cruise_ratio The ratio setting MINIMUM distance needed to be at a constant velocity
     * @param target The 1d end distance
     */
	public MotionProfiler(double cruiseV, double maxV, double acceleration, double cruise_ratio, double target, boolean neg){
		endDistance = target;
		if (cruiseV > maxV) {
			v_cruise = maxV;
		} else {
			v_cruise = cruiseV;
		}
		a = acceleration;
		cruiseRatio = cruise_ratio;
		negative = neg;
	}
	
	/**
     * Constructor. Creates a new MotionProfiling instance.
     *  
     * @param xvalues An array of all the x values the robot will cross
     * @param yvalues An array of all the y values the robot will cross
     * @param accel_distance The distance allotted for the robot to accelerate and decelerate
     * @param limitingFactor The amount you want to limit the max velocity to
     * 
     * ALERT: THIS ONLY WORKS FOR ARRAYS WITH AN EVEN NUMBER .length
     * 
     */
	
	public MotionProfiler(double[] xvalues, double[] yvalues, double acceleration, double max, double limitingFactor, double maxAcceleration) {
		ta = acceleration;
		x_values = xvalues;
		y_values = yvalues;
		limit= limitingFactor;
		max_velocity = max;
		max_acceleration = maxAcceleration;
	}
	
	/**
     * 1d motion profiling
     * 
     * @return An array of expected distance, velocity, and acceleration based on current time
     */
	public double[] execute1D(double time) {
	    	// The cruise ratio is the MINIMUM distance needed to be at cruise velocity
	    	double returnVelocity;
	    	double total;
	    	double returnAcceleration;
	    	double returnDistance;
	    	//calculating acceleration distance considering what portion of the distance cruise velocity will be at
	    	double accelDistance1 = (Math.pow(v_cruise, 2))/(2.0*a);
	    	if (accelDistance1 < (endDistance/2.0)*(1.0-(cruiseRatio/2.0))) {
	    		//below is only initiated if the acceleration distance allows for allotted cruise distance
	    		//calculating time taken for each segment
	    		double timeTakenAD = Math.sqrt((2.0*accelDistance1)/a);
	    		double cruiseTime = (endDistance - 2.0*accelDistance1)/v_cruise;
	    		double totalTime = 2.0*timeTakenAD + cruiseTime;
	    		total = totalTime;
	    		
	    		//returning values based on current time
	    		if (time <= timeTakenAD) {
	    			returnDistance = (1.0/2.0)*a*Math.pow(time, 2.0);
	    			returnVelocity = a*time;
	    			returnAcceleration = a;
	    		}
	    		else if (time > timeTakenAD && time < (totalTime - timeTakenAD)) {
	    			double t = time - timeTakenAD;
	    			returnDistance = accelDistance1 + t*v_cruise;
	    			returnVelocity = v_cruise;
	    			returnAcceleration = 0.0;
	    		}
	    		else {
	    			//the same properties that go for the acceleration period also go for the deceleration period
	    			double t = time - (cruiseTime+timeTakenAD);
	    			returnDistance = accelDistance1 + cruiseTime*v_cruise + (v_cruise*t+((1.0/2.0)*(-a)*Math.pow(t, 2.0)));
	    			returnVelocity = v_cruise + (-a)*t;
	    			returnAcceleration = -a;
	    		}
	    	}
	    	else {
	    		//lowers accel distance to fit the cruise ratio
	    		double accelDistance2 = endDistance*((1-cruiseRatio)/2);
	    		//setting a lower cruise velocity to account for the lower accel distance
	    		double v_cruise1 = Math.sqrt(2.0*a*accelDistance2);
	    		double timeTakenAD = Math.sqrt((2.0*accelDistance2)/a);
	    		double cruiseTime = (endDistance*cruiseRatio)/v_cruise1;
	    		double totalTimet = timeTakenAD*2.0 + cruiseTime;
	    		total = totalTimet;
	    		//Using the same calculations as before except with v_cruise1 instead of v_cruise
	    		if (time <= timeTakenAD) {
	    			returnDistance = (1.0/2.0)*a*Math.pow(time, 2.0);
	    			returnVelocity = a*time;
	    			returnAcceleration = a;
	    		}
	    		else if (time > timeTakenAD && time < (totalTimet - timeTakenAD)) {
	    			double t = time - timeTakenAD;
	    			returnDistance = accelDistance2 + t*v_cruise1;
	    			returnVelocity = v_cruise1;
	    			returnAcceleration = 0.0;
	    		}
	    		else {
	    			double t = time - (cruiseTime+timeTakenAD);
	    			returnDistance = accelDistance2 + cruiseTime*v_cruise1 + (v_cruise1*t+((1.0/2.0)*(-a)*Math.pow(t, 2)));
	    			returnVelocity = v_cruise1 + (-a)*t;
	    			returnAcceleration = -a;
	    		}
	    	}
	    	if (negative) {
	    		returnVelocity = returnVelocity * -1;
	    	}
	    	double distanceR;
	    	if (returnDistance>endDistance) {
	    		distanceR = endDistance;
	    	}
	    	else {
	    		distanceR = returnDistance;
	    	}
	    	//returning the current distance, velocity, and acceleration
	    	double[] array = {distanceR, returnVelocity, returnAcceleration, total};
	    	return array;
	    }
	
	public double[] getDistance(double currentDist, double increment) {
		double kale = execute1D(0.0)[3];
		double[] kaleDistance = new double[(int)(kale*100)+1];
		double[] kaleVelocity = new double[(int)(kale*100)+1];
		double[] kaleAcceleration = new double[(int)(kale*100)+1];
		for (double i=0; i<kale;i=i+.01) {
			kaleDistance[(int) (i*100)] = execute1D(i)[0];
			kaleVelocity[(int) (i*100)] = execute1D(i)[1];
			kaleAcceleration[(int) (i*100)] = execute1D(i)[2];
		}
		int l =0;
		int currentIndex=0;
		for (int i = 0; i < kaleDistance.length; i++) {
			//finding where in the path the robot currently is based on current time
			//This is where the array function below is used
			if (currentDist > kaleDistance[l]) {
				l++;
			}
			else {
				if (l!=0) {
					currentIndex = l-1;
				}
				else {
					currentIndex = 0;
				}
			}
		}
		
		double[] returnArray = {kaleDistance[currentIndex+(int)(increment*100)], kaleVelocity[currentIndex], kaleAcceleration[currentIndex], kaleDistance[kaleDistance.length-2]};
		return returnArray;
	}
	
	/**
     * Calculates values depending on the curvature of a three-point arc
     * 
     * @return An array of maximum velocity depending on curvature, distance covered (arc length), and the length of the chord between the arc
     */
	public double[] getMaxVelocity(double x_current, double y_current, double x_curve, double y_curve, double x_end, double y_end) {
		/*//These three inputs will come together to create an arc: an estimation of what the robot's motion will be
		//Finding the intersection between the chord and the perpendicular line, which used x_curve and y_curve to be created
		double x_inter = (x_end-x_current)/2.0;
		double y_inter = (y_end-y_current)/2.0;
		//finding the distance between the chord and perpendicular line intersection point and the x_curve and y_curve point
		double chord_height = Math.hypot(x_curve-x_inter, y_curve-y_inter);
		//finding the length of the chord
		double chord_length = Math.hypot(x_end-x_current, y_end-y_current);
		
		//finding the radius and arc length based on chord_height and chord_length
		double radius = (chord_height/2.0)+(Math.pow(chord_length, 2)/(8.0*chord_height));
		System.out.println(x_current+": "+x_curve+"; " + radius);
		//double radius = (Math.pow((chord_length/2.0), 2)/(2*chord_height))+Math.pow(chord_height, 2);
		double angle = 2*Math.acos((-chord_height/radius)+1);
		double distance = radius*angle;
		//2.0*Math.asin(chord_length/(2.0*radius));
		//calculating maximum velocity considering curvature of the arc
		double maxVelocity = Math.sqrt(radius*9.81);
		if (chord_height>chord_length) {
			System.out.println("That's why...");
		} */
		
		double left;
		double right;
		
		double x_inter = (x_end-x_current)/2.0;
		double y_inter = (y_end-y_current)/2.0;
		
		double ax = x_current;
		double ay = y_current;
		double bx = x_curve;
		double by = y_curve;
		double cx = x_end;
		double cy = y_end;
		
		double onecompa = 2.0*(cx-ax);
		double onecompb = 2.0*(cy-ay);
		double onecompc = -(Math.pow(ay, 2)+Math.pow(ax, 2)-Math.pow(cy, 2)-Math.pow(cx, 2));
		
		double twocompa = 2.0*(bx-ax);
		double twocompb = 2.0*(by-ay);
		double twocompc = -(Math.pow(ay, 2)+Math.pow(ax, 2)-Math.pow(by, 2)-Math.pow(bx, 2));
		
		double x_num = (onecompc*twocompb)-(twocompc*onecompb);
		double x_den = (onecompa*twocompb)-(twocompa*onecompb);
		double x = x_num/x_den;
		
		
		double y_num = (onecompa*twocompc)-(twocompa*onecompc);
		double y_den = (onecompa*twocompb)-(twocompa*onecompb);
		double y = y_num/y_den;
		
		double radius_right;
		double radius_left;
		boolean short_right;
		double radius_short = Math.hypot((x_current-x), (y_current-y))-(robot_width/2.0);
		double radius_long = Math.hypot((x_current-x), (y_current-y))+(robot_width/2.0);
		double distance_right;
		double distance_left;
		double chord_length_one_left;
		double chord_length_two_left;
		double chord_length_one_right;
		double chord_length_two_right;
		if (Math.abs(x_curve-x_inter)>Math.abs(y_curve-y_inter)) {
			if (x_curve>x_inter) {
				chord_length_one_left = (Math.hypot(x_curve-x_current, y_curve-y_current))-robot_width;
				chord_length_two_left = Math.hypot(x_end-x_curve, y_end-y_curve)-robot_width;
				chord_length_one_right = (Math.hypot(x_curve-x_current, y_curve-y_current))+robot_width;
				chord_length_two_right = Math.hypot(x_end-x_curve, y_end-y_curve)+robot_width;
				short_right= false;
			}
			else {
				chord_length_one_left = (Math.hypot(x_curve-x_current, y_curve-y_current))+robot_width;
				chord_length_two_left = Math.hypot(x_end-x_curve, y_end-y_curve)+robot_width;
				chord_length_one_right = (Math.hypot(x_curve-x_current, y_curve-y_current))-robot_width;
				chord_length_two_right = Math.hypot(x_end-x_curve, y_end-y_curve)-robot_width;
				short_right = true;
			}
		}
		else {
			if (y_curve>y_inter) {
				chord_length_one_left = (Math.hypot(x_curve-x_current, y_curve-y_current))-robot_width;
				chord_length_two_left = Math.hypot(x_end-x_curve, y_end-y_curve)-robot_width;
				chord_length_one_right = (Math.hypot(x_curve-x_current, y_curve-y_current))+robot_width;
				chord_length_two_right = Math.hypot(x_end-x_curve, y_end-y_curve)+robot_width;
				short_right= false;
			}
			else {
				chord_length_one_left = (Math.hypot(x_curve-x_current, y_curve-y_current))+robot_width;
				chord_length_two_left = Math.hypot(x_end-x_curve, y_end-y_curve)+robot_width;
				chord_length_one_right = (Math.hypot(x_curve-x_current, y_curve-y_current))-robot_width;
				chord_length_two_right = Math.hypot(x_end-x_curve, y_end-y_curve)-robot_width;
				short_right = true;
			}
		}
		
		if (short_right) {
			radius_right = radius_short;
			radius_left = radius_long;
		}
		else {
			radius_right = radius_long;
			radius_left = radius_short;
		}
		
		//RIGHT
		double angle_one_right = Math.acos(((2*Math.pow(radius_right, 2))-Math.pow(chord_length_one_right, 2))/(2*Math.pow(radius_right, 2)));
			//(2.0*radius*Math.asin(chord_length_one/(2.0*radius)))/(circ);
		double angle_two_right = Math.acos(((2*Math.pow(radius_right, 2))-Math.pow(chord_length_two_right, 2))/(2*Math.pow(radius_right, 2)));
		double distance_one_right = angle_one_right*radius_right;
		double distance_two_right = angle_two_right*radius_right;
		distance_right = distance_one_right+distance_two_right;
		//LEFT
		double angle_one_left = Math.acos(((2*Math.pow(radius_left, 2))-Math.pow(chord_length_one_left, 2))/(2*Math.pow(radius_left, 2)));
		//(2.0*radius*Math.asin(chord_length_one/(2.0*radius)))/(circ);
		double angle_two_left = Math.acos(((2*Math.pow(radius_left, 2))-Math.pow(chord_length_two_left, 2))/(2*Math.pow(radius_left, 2)));
		double distance_one_left = angle_one_left*radius_left;
		double distance_two_left = angle_two_left*radius_left;
		distance_left = distance_one_left+distance_two_left;
		double maxVelocity_right = Math.sqrt(radius_right*9.81);
		double maxVelocity_left = Math.sqrt(radius_left*9.81);
		
		
		/*if (x_curve>x_current) {
			left_one = false;
		} else {
			left_one = true;
		} if (y_curve>y_current) {
			back_one = false;
		} else {
			back_two = true;
		} if (x_end>x_curve) {
			left_two = true;
		} else {
			left_two = false;
		} if (y_end>y_curve) {
			back_two = true;
		} else {
			back_two = false;
		}*/
		
		
		
		double[] returnArray = {maxVelocity_right, maxVelocity_left, distance_right, distance_left};
		return returnArray;

	}
	
	/**
     * 2d motion profiling
     * 
     * @return An array of expected distance, velocity, and acceleration based on current time along with 
     * distance, velocity, and acceleration as specified by a specific x,y pair
     */
	public double[] execute2D(double time, double current_x, double current_y) {
		
		int allValues = x_values.length;
		
		double[] acceleration_left = new double[x_values.length+1];
		double[] acceleration_right = new double[x_values.length+1];
		double[] maxVelocities_left = new double[x_values.length+1];
		double[] maxVelocities_right = new double[x_values.length+1];
		double[] clampVel_left = new double[x_values.length+1];
		double[] clampVel_right = new double[x_values.length+1];
		double[] distance_left = new double[x_values.length];
		double[] distance_right = new double[x_values.length];
		double[] tempDistance_left = new double[x_values.length];
		double[] tempDistance_right = new double[x_values.length];
		double[] timeTaken_left = new double[x_values.length];
		double[] timeTaken_right = new double[x_values.length];

		
		maxVelocities_left[x_values.length] = 0.0;
		maxVelocities_right[x_values.length] = 0.0;
		
		for (int i = 0; i < allValues+1; i = i + 2) {
				//Sets the max velocity according to the next two points
				//Limit the max velocity by the limiting factor so one side of the robot doesn't go more than max speed
				if (i < allValues-1) {
					maxVelocities_left[i+1] = Clamp(0, max_velocity, (getMaxVelocity(x_values[i], y_values[i], x_values[i+1], y_values[i+1], x_values[i+2], y_values[i+2])[1]-limit));
					maxVelocities_right[i+1] = Clamp(0, max_velocity, (getMaxVelocity(x_values[i], y_values[i], x_values[i+1], y_values[i+1], x_values[i+2], y_values[i+2])[0]-limit));
					//Shows distance that WAS accomplished, why we add +1 to i
					tempDistance_left[i+1] = (getMaxVelocity(x_values[i], y_values[i], x_values[i+1], y_values[i+1], x_values[i+2], y_values[i+2])[3])/2;
					tempDistance_right[i+1] = (getMaxVelocity(x_values[i], y_values[i], x_values[i+1], y_values[i+1], x_values[i+2], y_values[i+2])[2])/2;
					if (i < (allValues-2)) {
						maxVelocities_left[i] = Clamp(0, max_velocity, (getMaxVelocity(x_values[i], y_values[i], x_values[i+1], y_values[i+1], x_values[i+2], y_values[i+2])[1]-limit));
						maxVelocities_right[i] = Clamp(0, max_velocity, (getMaxVelocity(x_values[i], y_values[i], x_values[i+1], y_values[i+1], x_values[i+2], y_values[i+2])[0]-limit));
						tempDistance_left[i] = (getMaxVelocity(x_values[i], y_values[i], x_values[i+1], y_values[i+1], x_values[i+2], y_values[i+2])[3])/2;
						tempDistance_right[i] = (getMaxVelocity(x_values[i], y_values[i], x_values[i+1], y_values[i+1], x_values[i+2], y_values[i+2])[2])/2;
					}
					//Use if statements so we don't get an error about array size
					else {
						maxVelocities_left[i] = Clamp(0, max_velocity, getMaxVelocity(x_values[i-2], y_values[i-2], x_values[i-1], y_values[i-1], x_values[i], y_values[i])[1]);
						maxVelocities_right[i] = Clamp(0, max_velocity, getMaxVelocity(x_values[i-2], y_values[i-2], x_values[i-1], y_values[i-1], x_values[i], y_values[i])[0]);
						tempDistance_left[i] = (getMaxVelocity(x_values[i-2], y_values[i-2], x_values[i-1], y_values[i-1], x_values[i], y_values[i])[3])/2;
						tempDistance_right[i] = (getMaxVelocity(x_values[i-2], y_values[i-2], x_values[i-1], y_values[i-1], x_values[i], y_values[i])[2])/2;
					}
				}
				else {
					maxVelocities_left[i] = Clamp(0, max_velocity, getMaxVelocity(x_values[i-2], y_values[i-2], x_values[i-1], y_values[i-1], x_values[i], y_values[i])[1]);
					maxVelocities_right[i] = Clamp(0, max_velocity, getMaxVelocity(x_values[i-2], y_values[i-2], x_values[i-1], y_values[i-1], x_values[i], y_values[i])[0]);
				}
				maxVelocities_left[allValues]=0;
				maxVelocities_right[allValues]=0;
				
				
		}

		double max_left = 0.0;
		double max_right = 0.0;
    	for (int i=0; i<allValues;i++) {
    		if (maxVelocities_left[i]>max_left && maxVelocities_left[i]<max_velocity) {
    			max_left = maxVelocities_left[i];
    		}
    		if (maxVelocities_right[i]>max_right && maxVelocities_right[i]<max_velocity) {
    			max_right = maxVelocities_right[i];
    		}
    	}
		
		//Making the distance array cumulative
		for (int i = 0; i< tempDistance_left.length;i++) {
			distance_left[i] = arraySum(tempDistance_left, i);
			distance_right[i] = arraySum(tempDistance_right, i);
		}
		
		//Sets acceleration depending on if two velocities are different
		//Acceleration period will most likely happen once every three points
		for (int i = 1; i < x_values.length; i++) {
			if (maxVelocities_left[i-1] != maxVelocities_left[i]) {
				double tdistance = distance_left[i]-distance_left[i-1];
				acceleration_left[i-1] = Clamp(Double.NEGATIVE_INFINITY, max_acceleration, (Math.pow(maxVelocities_left[i], 2)-Math.pow(maxVelocities_left[i-1], 2))/(2*tdistance));
				//acceleration is set for the current to the next point
			}
			if (maxVelocities_right[i-1] != maxVelocities_right[i]) {
				double tdistance = distance_right[i]-distance_right[i-1];
				acceleration_right[i-1] = Clamp(Double.NEGATIVE_INFINITY, max_acceleration, (Math.pow(maxVelocities_right[i], 2)-Math.pow(maxVelocities_right[i-1], 2))/(2*tdistance));
				//acceleration is set for the current to the next point
			}
		}
		
		//the start distance and max velocity will be zero
		distance_left[0] = 0;
		distance_right[0] = 0;
		
		//Calculating acceleration DISTANCE
		int y = 1;
		int z = 1;
		int accelIndex_left = 0;
		int accelIndex_right = 0;
		for (int i = 1; i<x_values.length; i++) {
			//Calculating velocity output after a specified distance of acceleration
			double v_left = Math.sqrt(2*ta*distance_left[y]);
			double v_right = Math.sqrt(2*ta*distance_right[z]);
			//Checking if that velocity is significantly less than the maxVelocity set for that distance
			if (v_left < maxVelocities_left[y]) {
				y = y + 1;
			}
			else {
				//Subtracting one because we don't want the accelerated velocity to be OVER max velocity
				accelIndex_left = y-1;
			}
			if (v_right < maxVelocities_right[z]) {
				z = z + 1;
			}
			else {
				//Subtracting one because we don't want the accelerated velocity to be OVER max velocity
				accelIndex_right = z-1;
			}
		}
		if (accelIndex_left==0 || accelIndex_right==0) {
			System.out.println("lower distance between points or increase acceleration");
		}  
		//SETTING ACCELERATION
		for (int i = 0; i < (accelIndex_left+1); i++) {
			if (acceleration_left[i] < ta) {
				acceleration_left[i] = ta;
			}
			else {
				//do nothing
			}
		}
		for (int i = 0; i < (accelIndex_right+1); i++) {
			if (acceleration_right[i] < ta) {
				acceleration_right[i] = ta;
			}
			else {
				//do nothing
			}
		}
		
		
		//The velocities set below will override those previously set in the maxVelocities array based on acceleration, effectively clamping them
    	/*for (int i = 1; i < (accelIndex+1); i++) {
    		//Sets the current velocity during the start acceleration period
    		maxVelocities[i] = Clamp(Double.NEGATIVE_INFINITY, max_velocity, Math.sqrt(Math.pow(maxVelocities[i-1], 2)+2*acceleration[i-1]*(distance[i]-distance[i-1])));
    	}*/
    	
    	//SETTING DECELERATION
    	//We want this to be an overapproximation instead of an underapproximation so it fully goes to 0
    	int decelIndex_left= accelIndex_left+1;
    	int decelIndex_right= accelIndex_right+1;
    	for (int j=0; j<allValues; j++) {
    		for (int i = (allValues-decelIndex_left); i < (allValues); i++) {
    			//Sets the deceleration values
    			double endDistance = distance_left[allValues-1]-distance_left[allValues-decelIndex_left-1];
    			//making sure acceleration isn't greater than the max acceleration specified
    			acceleration_left[i] = (-1.0*Math.pow(maxVelocities_left[(allValues-decelIndex_left)],2))/(2*endDistance);
    			if (-acceleration_left[i]>max_acceleration) {
    				decelIndex_left++;
    			}
    			else {
    				//do nothing
    			}
    		}
    		for (int i = (allValues-decelIndex_right); i < (allValues); i++) {
    			//Sets the deceleration values
    			double endDistance = distance_right[allValues-1]-distance_right[allValues-decelIndex_right-1];
    			//making sure acceleration isn't greater than the max acceleration specified
    			acceleration_right[i] = (-1.0*Math.pow(maxVelocities_right[(allValues-decelIndex_right)],2))/(2*endDistance);
    			if (-acceleration_right[i]>max_acceleration) {
    				decelIndex_right++;
    			}
    			else {
    				//do nothing
    			}
    		}
    	}

    	
    	
    	//Changes the current velocity during the deceleration period
    	if ((allValues-decelIndex_left)==0) {
    		decelIndex_left = decelIndex_left-1;
    	}
    	if ((allValues-decelIndex_right)==0) {
    		decelIndex_right = decelIndex_right-1;
    	}
    	for (int i = (allValues-decelIndex_left-1); i < (allValues); i++) {
    		maxVelocities_left[i] = Clamp(Double.NEGATIVE_INFINITY, max_left, Math.sqrt(Math.pow(maxVelocities_left[i-1], 2)+2*acceleration_left[i-1]*(distance_left[i]-distance_left[i-1])));
    		//System.out.println(maxVelocities[i]);
    	}
    	for (int i = (allValues-decelIndex_right-1); i < (allValues); i++) {
    		maxVelocities_right[i] = Clamp(Double.NEGATIVE_INFINITY, max_right, Math.sqrt(Math.pow(maxVelocities_right[i-1], 2)+2*acceleration_right[i-1]*(distance_right[i]-distance_right[i-1])));
    		//System.out.println(maxVelocities[i]);
    	}
    	
    	
    	//Making sure the CHANGE of acceleration isn't greater than max acceleration specified
    	for (int i=1; i<allValues;i++) {
    		double check_left = acceleration_left[i]-acceleration_left[i-1];
    		
    		if (Math.abs(check_left)>max_acceleration) {
    			int h=i-1;
    			boolean hm = false;
    			for (int m=0; m<allValues;m++) {
    				for (int b=(h); b<(i+1);b++) {
    					acceleration_left[b] = check_left/h;
    					if (Math.abs(acceleration_left[b]-acceleration_left[i-1])>max_acceleration) {
    						hm = true;
    					}
    					else {
    						hm = false;
    					}
    				} if (hm) {
						h=h-1;
					}
    			}
    		}
    		
    	}
    	for (int i=1; i<allValues;i++) {
    		double check_right = acceleration_right[i]-acceleration_right[i-1];
    		if (Math.abs(check_right)>max_acceleration) {
    			int h=i-1;
    			boolean hm = false;
    			for (int m=0; m<allValues;m++) {
    				for (int b=(h); b<(i+1);b++) {
    					acceleration_right[b] = check_right/h;
    					if (Math.abs(acceleration_right[b]-acceleration_right[i-1])>max_acceleration) {
    						hm = true;
    					}
    					else {
    						hm = false;
    					}
    				} if (hm) {
						h=h-1;
					}
    			}
    		}
    	}
    	
    	
    	double[] velocitiestwo_left = new double[allValues];
    	double[] velocitiestwo_right = new double[allValues];
    	double[] newAccel_left = new double[allValues];
    	double[] newAccel_right = new double[allValues];
    	velocitiestwo_left[0]=0;
    	velocitiestwo_right[0]=0;
    	if (maxVelocities_left[allValues-1]>0) {
    		int l=1;
    		double red = 0.0;
    		for (int r=0; r<1000;r++) {
    			for (int i=allValues-l; i<allValues; i++) {
    			//(Math.abs((acceleration[i]-red)-acceleration[i-1])<max_acceleration) && 
    				if (Math.abs((acceleration_left[i]-red)-acceleration_left[i-1])<max_acceleration && Math.abs(acceleration_left[i]-red)<max_acceleration) {
    					newAccel_left[i]=acceleration_left[i]-red;  	
    				}
    				else {
    					if (newAccel_left[i] < 100) {
    						newAccel_left[i] = newAccel_left[i];
    					}
    					else {
    						newAccel_left[i] = acceleration_left[i];
    					}
    				}
    			}	
    			for (int i=0; i<allValues-l;i++) {
    				newAccel_left[i] = acceleration_left[i];
    			}
    			for (int m = 1; m < (allValues); m++) {
    				velocitiestwo_left[m] = Clamp(Double.NEGATIVE_INFINITY, max_left, Math.sqrt(Math.pow(velocitiestwo_left[m-1], 2)+2*newAccel_left[m-1]*(distance_left[m]-distance_left[m-1])));
    			}
    			if (velocitiestwo_left[allValues-1]>.1) {
    				if (l<(allValues-1)) {
    					l=l+1;
    				}
    				else {
    					red = red+.01;
    					//l=1;
    				}
    				
    			}
    			else if (velocitiestwo_left[allValues-1]<=.1) {
    				
    			}
    			else {
    				red=red-.001;
    			}
    		}
    	}
    	
    	
    	
    	if (maxVelocities_right[allValues-1]>0) {
    		int l=1;
    		double red = 0.0;
    		for (int r=0; r<1000;r++) {
    			for (int i=allValues-l; i<allValues; i++) {
    			//(Math.abs((acceleration[i]-red)-acceleration[i-1])<max_acceleration) && 
    				if (Math.abs((acceleration_right[i]-red)-acceleration_right[i-1])<max_acceleration && Math.abs(acceleration_right[i]-red)<max_acceleration) {
    					newAccel_right[i]=acceleration_right[i]-red;  	
    				}
    				else {
    					if (newAccel_right[i] < 100) {
    						newAccel_right[i] = newAccel_right[i];
    					}
    					else {
    						newAccel_right[i] = acceleration_right[i];
    					}
    				}
    			}	
    			for (int i=0; i<allValues-l;i++) {
    				newAccel_right[i] = acceleration_right[i];
    			}
    			for (int m = 1; m < (allValues); m++) {
    				velocitiestwo_right[m] = Clamp(Double.NEGATIVE_INFINITY, max_right, Math.sqrt(Math.pow(velocitiestwo_right[m-1], 2)+2*newAccel_right[m-1]*(distance_right[m]-distance_right[m-1])));
    			}
    			if (velocitiestwo_right[allValues-1]>.1) {
    				if (l<(allValues-1)) {
    					l=l+1;
    				}
    				else {
    					red = red+.01;
    					//l=1;
    				}
    				
    			}
    			else if (velocitiestwo_right[allValues-1]<=.1) {

    			}
    			else {
    				red=red-.001;
    			}
    		}
    	}
    	
    	for (int i=0; i<velocitiestwo_right.length;i++) {
    		maxVelocities_left[i] = velocitiestwo_left[i];
    		acceleration_right[i] = newAccel_right[i];
    		maxVelocities_right[i] = velocitiestwo_right[i];
    		acceleration_left[i] = newAccel_left[i];
    	}
    	
    	
    	/*for (int i=1; i<allValues; i++) {
    		maxVelocities[i] = Clamp(Double.NEGATIVE_INFINITY, max_velocity, Math.sqrt(Math.pow(maxVelocities[i-1], 2)+2*acceleration[i-1]*(distance[i]-distance[i-1])));
    		
    	}*/
		
    	
    	//Clamping velocity values
    	for (int i=0; i<allValues; i++) {
    	    clampVel_left[i] = Clamp(Double.NEGATIVE_INFINITY, max_left, maxVelocities_left[i]);
    	    clampVel_right[i] = Clamp(Double.NEGATIVE_INFINITY, max_right, maxVelocities_right[i]);
    	}
    	
    	
		
	
    	//Setting time
		timeTaken_left[0] = 0;
		timeTaken_right[0] = 0;
    	for (int i = 1; i < x_values.length; i++) {
    		//Change in distance
    		//LEFT
    		double temp_left = distance_left[i] - distance_left[i-1];
    		double temp_right = distance_right[i] - distance_right[i-1];
    		if (i<(allValues)) {
 
    			if (acceleration_left[i-1] != 0) {
    				//the timeTaken array measures time COMPLETED whereas the velocity and acceleration array outputs FUTURE values (for the next increment)
    				//This is why we subtract i by 1 for velocity and acceleration
    				timeTaken_left[i] = (2*temp_left)/(clampVel_left[i-1]+clampVel_left[i]);
    			}
    			
    			else {
    				//a reformat of d = v*t; there is no acceleration
    				timeTaken_left[i] = temp_left/clampVel_left[i];
    			}
    			if (acceleration_right[i-1] != 0) {
    				//the timeTaken array measures time COMPLETED whereas the velocity and acceleration array outputs FUTURE values (for the next increment)
    				//This is why we subtract i by 1 for velocity and acceleration
    				timeTaken_right[i] = (2*temp_right)/(clampVel_right[i-1]+clampVel_right[i]);
    			}
    			
    			else {
    				//a reformat of d = v*t; there is no acceleration
    				timeTaken_right[i] = temp_right/clampVel_right[i];
    			}
    		}
    		else {
    			timeTaken_left[i] = (2*temp_left)/(clampVel_left[i-1]);
    			timeTaken_right[i] = (2*temp_right)/(clampVel_right[i-1]);
    	
    		}
    	}
    	
    	
    	double[] velocitiesthree = new double[allValues];
    	double[] accelerationthree=new double[allValues];
    	boolean left;
    	if (arraySum(timeTaken_left, timeTaken_left.length-1)>arraySum(timeTaken_right, timeTaken_right.length-1)) {
    		double green = 0.0;
    		double blue = 0.0;
    		left = true;
    		for (int i=0; i<1000; i++) {
    			for (int u=0; u<allValues;u++) {
    				if (clampVel_right[u]>green) {
    					velocitiesthree[u] = clampVel_right[u]-green;
    				}
    			}
    			for (int u=0; u<accelIndex_right;u++) {
    				if (velocitiesthree[u]>0) {
    					velocitiesthree[u] = velocitiesthree[u]+blue;
    				}
    			}
    			for (int u=0; u<(allValues-accelIndex_right-1);u++) {
    				if (velocitiesthree[u]>blue) {
    					velocitiesthree[u] = velocitiesthree[u]-blue;
    				}
    			}
    			for (int u=0; u<allValues; u++) {
    				double temp_right = distance_right[u] - distance_right[u-1];
    				if (u<(allValues)) {
    					if (acceleration_right[u-1] != 0) {
    						timeTaken_right[u] = (2*temp_right)/(velocitiesthree[u-1]+velocitiesthree[u]);
    					} else {
    						timeTaken_right[u] = temp_right/velocitiesthree[u];
    					}
    				} else {
    					timeTaken_right[u] = (2*temp_right)/(velocitiesthree[u-1]);
    				}
    			}
    			if (Math.abs(arraySum(timeTaken_right, timeTaken_right.length-1)-arraySum(timeTaken_left, timeTaken_left.length-1))>.01) {
    				green = green+.01;
    				System.out.println(green);
    			}
    			else if (velocitiesthree[allValues-1]>.01) {
    				
    				blue = blue+.01;
    			}	
    			else {
    				for (int k=0; k<allValues;k++) {
    					double tempTime = timeTaken_right[k+1]-timeTaken_right[k];
    					double tempDistance = distance_right[k+1]-distance_right[k];
    					accelerationthree[k] = (2.0*(tempDistance-velocitiesthree[k]*tempTime))/Math.pow(tempTime, 2);
    					System.out.println(k + ": " + timeTaken_right[k]);
    					
    				}
    			}
    		}
    	}
    	else {
    		double green = 0.0;
    		double blue = 0.0;
    		left = false;
    		for (int i=0; i<1000; i++) {
    			for (int u=0; u<allValues;u++) {
    				if (clampVel_left[u]>green) {
    					velocitiesthree[u] = clampVel_left[u]-green;
    				}
    			}
    			for (int u=0; u<accelIndex_left;u++) {
    				if (velocitiesthree[u]>0) {
    					velocitiesthree[u] = velocitiesthree[u]+blue;
    				}
    			}
    			for (int u=0; u<(allValues-accelIndex_left-1);u++) {
    				if (velocitiesthree[u]>blue) {
    					velocitiesthree[u] = velocitiesthree[u]-blue;
    				}
    			}
    			for (int u=1; u<allValues;u++) {
    				double temp_left = distance_left[u] - distance_left[u-1];
    				if (u<(allValues)) {
    					if (acceleration_left[u-1] != 0) {
    						timeTaken_left[u] = (2*temp_left)/(velocitiesthree[u-1]+velocitiesthree[u]);
    					} else {
    						timeTaken_left[u] = temp_left/velocitiesthree[u];
    					}
    				} else {
    					timeTaken_left[u] = (2*temp_left)/(velocitiesthree[u-1]);
    				}
    			}
    			
    			if (Math.abs(arraySum(timeTaken_right, timeTaken_right.length-1)-arraySum(timeTaken_left, timeTaken_left.length-1))>.001) {
    				green = green+.001;
    			}
    			
    			else if (velocitiesthree[allValues-1]>.01) {
    				
    				blue = blue+.01;
    			}
    			else {
    				for (int k=0; k<(allValues-1);k++) {
    					double tempTime = timeTaken_left[k+1];
    					double tempDistance = distance_left[k+1]-distance_left[k];
    					accelerationthree[k] = (2.0*(tempDistance-velocitiesthree[k]*tempTime))/Math.pow(tempTime, 2);
    				}
    				
    			}
    		}
    	}
		double[] newVelocities_left = new double[allValues];
		double[] newVelocities_right = new double[allValues];
		double[] newAcceleration_left = new double[allValues];
		double[] newAcceleration_right = new double[allValues];
    	if (left) {
    		for (int i=0; i<(allValues);i++) {
    			if (i<(allValues-1)) {
    				newAcceleration_right[i] = accelerationthree[i];
    			}
    			newVelocities_right[i] = velocitiesthree[i];
    			newAcceleration_left[i] = acceleration_left[i];
    			newVelocities_left[i] = maxVelocities_left[i];
    		}
    	}
    	else {
    		for (int i=0; i<(allValues);i++) {
    			if (i<(allValues-1)) {
    				newAcceleration_left[i] = accelerationthree[i];
    			}
    			newVelocities_left[i] = velocitiesthree[i];
    			newVelocities_right[i] = maxVelocities_right[i];
    			newAcceleration_right[i] = acceleration_right[i];
    		}
    	}
    	//System.out.println(arraySum(timeTaken_left, timeTaken_left.length-1));
    	//establishing return values
    	double returnDistance_left = 0;
    	double returnVelocity_left = 0;
    	double returnAccel_left = 0;
    	double returnDistance_right = 0;
    	double returnVelocity_right = 0;
    	double returnAccel_right = 0;
    	
    	//LEFT
		int currentIndex_left = 0;
		int l_left = 0;
		for (int i = 0; i < allValues; i++) {
			//finding where in the path the robot currently is based on current time
			//This is where the array function below is used
			if (time > arraySum(timeTaken_left, l_left)) {
				l_left = l_left+1;
				
			}
			else {
				if (l_left!=0) {
					currentIndex_left = l_left-1;
				}
				else {
					currentIndex_left = 0;
				}
			}
		}
		if (time > arraySum(timeTaken_left, timeTaken_left.length-1)) {
			currentIndex_left = timeTaken_left.length-1;
		}
		//RIGHT
		int currentIndex_right = 0;
		int l_right = 0;
		for (int i = 0; i < allValues; i++) {
			//finding where in the path the robot currently is based on current time
			//This is where the array function below is used
			if (time > arraySum(timeTaken_right, l_right)) {
				l_right = l_right+1;
			}
			else {
				if (l_right!=0) {
					currentIndex_right = l_right-1;
				}
				else {
					currentIndex_right = 0;
				}
			}
		}
		if (time > arraySum(timeTaken_right, timeTaken_right.length-1)) {
			currentIndex_right = timeTaken_right.length-1;
		}
		
		
		/*tempTestDistance[0]=0;
		for (int i=1; i<allValues; i++) {
			double timeDiff = timeTaken[i];
			tempTestDistance[i] = (1.0/2.0)*acceleration[i-1]*Math.pow(timeDiff, 2)+clampVel[i-1]*timeDiff;
		}
		for (int i=0; i<allValues; i++) {
			testDistance[i] = arraySum(tempTestDistance, i);
			if (i==(allValues-1)) {
				//System.out.println(Math.abs(distance[distance.length-1]-testDistance[i]) + " and " + testDistance[i]);
			}
			
		}*/
		
		
		double tempAccel_left = newAcceleration_left[currentIndex_left];
		double tempVel_left = newVelocities_left[currentIndex_left];
		double tempAccel_right = newAcceleration_right[currentIndex_right];
		double tempVel_right = newVelocities_right[currentIndex_right];
		

		
		//LEFT
		//calculating the different in time and distance (between current position and last increment)
		double timeDifference_left = time - arraySum(timeTaken_left, currentIndex_left);
		double distanceChange_left = Math.abs(tempVel_left*timeDifference_left+(0.5)*tempAccel_left*Math.pow(timeDifference_left, 2));
		//System.out.println("distance change: " + maxVelocities[currentIndex] + "; second part: " + (0.5)*acceleration[currentIndex]*Math.pow(timeDifference, 2));
		double velocityChange_left = Math.abs(tempVel_left*timeDifference_left+(0.5)*tempAccel_left*Math.pow(timeDifference_left, 2));
		//calculating total distance covered
		returnDistance_left = distance_left[currentIndex_left]+distanceChange_left;

		//RIGHT
		double timeDifference_right = time - arraySum(timeTaken_right, currentIndex_right);
		double distanceChange_right = Math.abs(tempVel_right*timeDifference_right+(0.5)*tempAccel_right*Math.pow(timeDifference_right, 2));
		double velocityChange_right = Math.abs(tempVel_right*timeDifference_right+(0.5)*tempAccel_right*Math.pow(timeDifference_right, 2));
		returnDistance_right = distance_right[currentIndex_right]+distanceChange_right;
		
		//LEFT
		if (tempAccel_left!=0.0) {
			returnAccel_left = newAcceleration_left[currentIndex_left];
			//calculating predicted velocity that the robot is currently at based on change in distance covered
			double squared = 2*returnAccel_left*velocityChange_left+Math.pow(tempVel_left, 2);
			//recalculating velocity, just in case
			if (squared<0) {
				//Clamping
				returnVelocity_left = Clamp(Double.NEGATIVE_INFINITY, max_left, -1.0*Math.sqrt(-squared))-limit;
			}
			else {
				returnVelocity_left = Clamp(Double.NEGATIVE_INFINITY, max_left, (Math.sqrt(squared)))-limit;
			}
		}
		else {
			returnAccel_left = 0.0;
			//velocity should stay the same if acceleration is 0
			if (newVelocities_left[currentIndex_left]>limit) {
				returnVelocity_left = tempVel_left-limit;
			}
			else {
				returnVelocity_left = tempVel_left;
			}
		}
		//RIGHT
		if (tempAccel_right!=0.0) {
			returnAccel_right = newAcceleration_right[currentIndex_right];
			//calculating predicted velocity that the robot is currently at based on change in distance covered
			double squared = 2*returnAccel_right*velocityChange_right+Math.pow(tempVel_right, 2);
			//recalculating velocity, just in case
			if (squared<0) {
				//Clamping
				returnVelocity_right = Clamp(Double.NEGATIVE_INFINITY, max_right, -1.0*Math.sqrt(-squared))-limit;
			}
			else {
				returnVelocity_right = Clamp(Double.NEGATIVE_INFINITY, max_right, (Math.sqrt(squared)))-limit;
			}
		}
		else {
			returnAccel_right = 0.0;
			//velocity should stay the same if acceleration is 0
			if (newVelocities_right[currentIndex_right]>limit) {
				returnVelocity_right = tempVel_right-limit;
			}
			else {
				returnVelocity_right = tempVel_right;
			}
		}
		
		
		//stores what the distance covered, current velocity, and current acceleration values should be in an array
		//stuff that begins with 'return' is from time, stuff that begins with 'current' is for location
		double[] returnArray = {returnDistance_left, returnVelocity_left, returnAccel_left, arraySum(timeTaken_left, timeTaken_left.length-1), returnDistance_right, returnVelocity_right, returnAccel_right, arraySum(timeTaken_right, timeTaken_right.length-1)};
		return returnArray;
	}
	
	/**
     * Calculates velocities, acceleration based on current distance
     * 
     * This is probably the function that will be used during autonomous (for the 2D Motion Profiling)
     * 
     * @param currentDist The instantaneous distance
     * 
     * @return returnArray An array of the current Velocity, acceleration, and total distance that will be covered
     */
	public double[] getVelocity(double currentDist, double increment) {
		//Getting max time to get to the end
		double kale = execute2D(0.0, 0.0, 0.0)[3];
		double[] kaleDistance = new double[(int)(kale*100)+1];
		double[] kaleVelocity = new double[(int)(kale*100)+1];
		double[] kaleAcceleration = new double[(int)(kale*100)+1];
		//going through time to create some sort of a timeline
		for (double i=0; i<kale;i=i+.01) {
			//Adding it to an array
			kaleDistance[(int) (i*100)] = execute2D(i, 0.0, 0.0)[0];
			kaleVelocity[(int) (i*100)] = execute2D(i, 0.0, 0.0)[1];
			kaleAcceleration[(int) (i*100)] = execute2D(i, 0.0, 0.0)[2];
		}
		int l =0;
		int currentIndex=0;
		//Finding where on the timeline you are based on current distance
		//utilizing time to find velocity and acceleration
		for (int i = 0; i < kaleDistance.length; i++) {
			if (currentDist > kaleDistance[l]) {
				l++;
			}
			else {
				if (l!=0) {
					currentIndex = l-1;
				}
				else {
					currentIndex = 0;
				}
			}
		}
		//returning the values as well as the target distance to help with debugging
		double[] returnArray = {kaleDistance[currentIndex+(int)(increment*100)], kaleVelocity[currentIndex], kaleAcceleration[currentIndex], kaleDistance[kaleDistance.length-2]};
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
	
	/**
     * Limits the output to a specified range
     * 
     * @param min The minimum range
     * @param max The maximum range
     * @param value The output to be limited
     * @return The limited values
     */
    public double Clamp(double min, double max, double value) {
    	    if (value < min) {
    	        value = min;
    	    }
    	    if (value > max) {
    	        value = max;
    	    }
    	    return value;
    }
	 
    
 
}

