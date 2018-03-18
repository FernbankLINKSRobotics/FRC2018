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
	
	/**
     * Calculates velocities, acceleration based on current distance
     * 
     * This is probably the function that will be used during autonomous (for the 2D Motion Profiling)
     * 
     * @param currentDist The instantaneous angle
     * @param increment How far in the future to pull the angle
     * 
     * @return returnArray An array of the next angle, velocity, acceleration, and total angle that will be covered
     */
	public double[] getDistance(double currentDist, double increment) {
		double kale = execute1D(0.0)[3];
		double[] kaleDistance = new double[(int)(kale*100)+1];
		double[] kaleVelocity = new double[(int)(kale*100)+1];
		double[] kaleAcceleration = new double[(int)(kale*100)+1];
		for (double i=0; i<kale;i+=.01) {
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
		
		
		double radius = Math.hypot((x_current-x), (y_current-y));
		double distance;
		double chord_length_one = Math.hypot(x_curve-x_current, y_curve-y_current);
		double chord_length_two = Math.hypot(x_end-x_curve, y_end-y_curve);
		double angle_one = Math.acos(((2*Math.pow(radius, 2))-Math.pow(chord_length_one, 2))/(2*Math.pow(radius, 2)));
			//(2.0*radius*Math.asin(chord_length_one/(2.0*radius)))/(circ);
		double angle_two = Math.acos(((2*Math.pow(radius, 2))-Math.pow(chord_length_two, 2))/(2*Math.pow(radius, 2)));
		double distance_one = angle_one*radius;
		double distance_two = angle_two*radius;
		distance = distance_one+distance_two;
		double maxVelocity = Math.sqrt(radius*9.81);
		
		
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
		
		
		
		double[] returnArray = {maxVelocity, Math.abs(distance), chord_length_one};
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
		
		double[] acceleration = new double[x_values.length+1];
		double[] maxVelocities = new double[x_values.length+1];
		double[] clampVel = new double[x_values.length+1];
		double[] distance = new double[x_values.length];
		double[] tempDistance = new double[x_values.length];
		double[] testDistance = new double[x_values.length];
		double[] tempTestDistance = new double[x_values.length];
		double[] timeTaken = new double[x_values.length];

		
		maxVelocities[x_values.length] = 0.0;
		
		for (int i = 0; i < allValues+1; i = i + 2) {
				//Sets the max velocity according to the next two points
				//Limit the max velocity by the limiting factor so one side of the robot doesn't go more than max speed
				if (i < allValues-1) {
					maxVelocities[i+1] = Clamp(0, max_velocity, (getMaxVelocity(x_values[i], y_values[i], x_values[i+1], y_values[i+1], x_values[i+2], y_values[i+2])[0]-limit));
					//Shows distance that WAS accomplished, why we add +1 to i
					tempDistance[i+1] = (getMaxVelocity(x_values[i], y_values[i], x_values[i+1], y_values[i+1], x_values[i+2], y_values[i+2])[1])/2;
					if (i < (allValues-2)) {
						maxVelocities[i] = Clamp(0, max_velocity, (getMaxVelocity(x_values[i], y_values[i], x_values[i+1], y_values[i+1], x_values[i+2], y_values[i+2])[0]-limit));
						tempDistance[i] = (getMaxVelocity(x_values[i], y_values[i], x_values[i+1], y_values[i+1], x_values[i+2], y_values[i+2])[1])/2;
					}
					//Use if statements so we don't get an error about array size
					else {
						maxVelocities[i] = Clamp(0, max_velocity, getMaxVelocity(x_values[i-2], y_values[i-2], x_values[i-1], y_values[i-1], x_values[i], y_values[i])[0]);
						tempDistance[i] = (getMaxVelocity(x_values[i-2], y_values[i-2], x_values[i-1], y_values[i-1], x_values[i], y_values[i])[1])/2;
					}
				}
				else {
					maxVelocities[i] = Clamp(0, max_velocity, getMaxVelocity(x_values[i-2], y_values[i-2], x_values[i-1], y_values[i-1], x_values[i], y_values[i])[0]);
				}
				maxVelocities[allValues]=0;
				
				
		}
		
		maxVelocities[allValues] = 0;
		
		//Making the distance array cumulative
		for (int i = 0; i< tempDistance.length;i++) {
			distance[i] = arraySum(tempDistance, i);
		}
		
		//Sets acceleration depending on if two velocities are different
		//Acceleration period will most likely happen once every three points
		for (int i = 1; i < x_values.length; i++) {
			if (maxVelocities[i-1] != maxVelocities[i]) {
				double tdistance = distance[i]-distance[i-1];
				acceleration[i-1] = Clamp(Double.NEGATIVE_INFINITY, max_acceleration, (Math.pow(maxVelocities[i], 2)-Math.pow(maxVelocities[i-1], 2))/(2*tdistance));
				//acceleration is set for the current to the next point
			}
		}
		
		//the start distance and max velocity will be zero
		distance[0] = 0;
		maxVelocities[0] = Math.sqrt(2.0*a*distance[1]);
		
		//Calculating acceleration DISTANCE
		int y = 1;
		int accelIndex = 0;
		for (int i = 1; i<x_values.length; i++) {
			//Calculating velocity output after a specified distance of acceleration
			double v = Math.sqrt(2*ta*distance[y]);
			//Checking if that velocity is significantly less than the maxVelocity set for that distance
			if (v < maxVelocities[y]) {
				y = y + 1;
			}
			else {
				//Subtracting one because we don't want the accelerated velocity to be OVER max velocity
				accelIndex = y-1;
			}
		}
		//SETTING ACCELERATION
		for (int i = 0; i < (accelIndex+1); i++) {
			if (acceleration[i] < ta) {
				acceleration[i] = ta;
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
    	int decelIndex= accelIndex+1;
    	for (int j=0; j<allValues; j++) {
    		for (int i = (allValues-decelIndex); i < (allValues); i++) {
    			//Sets the deceleration values
    			double endDistance = distance[allValues-1]-distance[allValues-decelIndex-1];
    			//making sure acceleration isn't greater than the max acceleration specified
    			acceleration[i] = (-1.0*Math.pow(maxVelocities[(allValues-decelIndex)],2))/(2*endDistance);
    			if (-acceleration[i]>max_acceleration) {
    				decelIndex++;
    			}
    			else {
    				//do nothing
    			}
    		}
    	}
    	
    	
    	//Changes the current velocity during the deceleration period
    	if ((allValues-decelIndex)==0) {
    		decelIndex = decelIndex-1;
    	}
    	for (int i = (allValues-decelIndex-1); i < (allValues); i++) {
    		maxVelocities[i] = Clamp(Double.NEGATIVE_INFINITY, max_velocity, Math.sqrt(Math.pow(maxVelocities[i-1], 2)+2*acceleration[i-1]*(distance[i]-distance[i-1])));
    		
    	}
    	for (int i=0; i<allValues;i++) {
    	}
    	
    	
    	//Making sure the CHANGE of acceleration isn't greater than max acceleration specified
    	for (int i=1; i<allValues;i++) {
    		double check = acceleration[i]-acceleration[i-1];
    		if (Math.abs(check)>max_acceleration) {
    			int h=i-1;
    			boolean hm = false;
    			for (int m=0; m<allValues;m++) {
    				for (int b=(h); b<(i+1);b++) {
    					acceleration[b] = check/h;
    					if (Math.abs(acceleration[b]-acceleration[i-1])>max_acceleration) {
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
    	
    	double[] velocitiestwo = new double[allValues];
    	double[] newAccel = new double[allValues];
    	velocitiestwo[0]=0;
    	if (maxVelocities[allValues-1]>0) {
    		int l=1;
    		double red = 0.0;
    		for (int r=0; r<1000;r++) {
    			for (int i=allValues-l; i<allValues; i++) {
    			//(Math.abs((acceleration[i]-red)-acceleration[i-1])<max_acceleration) && 
    				if (Math.abs((acceleration[i]-red)-acceleration[i-1])<max_acceleration && Math.abs(acceleration[i]-red)<max_acceleration) {
    					newAccel[i]=acceleration[i]-red;  	
    				}
    				else {
    					if (newAccel[i] < 100) {
    						newAccel[i] = newAccel[i];
    					}
    					else {
    						newAccel[i] = acceleration[i];
    					}
    				}
    			}	
    			for (int i=0; i<allValues-l;i++) {
    				newAccel[i] = acceleration[i];
    			}
    			for (int m = 1; m < (allValues); m++) {
    				velocitiestwo[m] = Clamp(Double.NEGATIVE_INFINITY, max_velocity, Math.sqrt(Math.pow(velocitiestwo[m-1], 2)+2*newAccel[m-1]*(distance[m]-distance[m-1])));
    			}
    			if (velocitiestwo[allValues-1]>.1) {
    				if (l<(allValues-1)) {
    					l=l+1;
    				}
    				else {
    					red = red+.01;
    					//l=1;
    				}
    				
    			}
    			else if (velocitiestwo[allValues-1]<=.1) {
    			}
    			else {
    				red=red-.001;
    			}
    			
    			
    		}
    		
    	}
    	
    	for (int i=0; i<velocitiestwo.length;i++) {
    		maxVelocities[i] = velocitiestwo[i];
    		acceleration[i] = newAccel[i];
    		System.out.println(maxVelocities[i] +", " + acceleration[i]);
    	}
    	
    	
    	/*for (int i=1; i<allValues; i++) {
    		maxVelocities[i] = Clamp(Double.NEGATIVE_INFINITY, max_velocity, Math.sqrt(Math.pow(maxVelocities[i-1], 2)+2*acceleration[i-1]*(distance[i]-distance[i-1])));
    		
    	}*/
		
    	//Clamping velocity values
    	for (int i=0; i<maxVelocities.length; i++) {
    	    clampVel[i] = Clamp(Double.NEGATIVE_INFINITY, max_velocity, maxVelocities[i]);
    	}
    	
    	//Setting time
		timeTaken[0] = 0;
    	for (int i = 1; i < x_values.length; i++) {
    		//Change in distance
    		double temp = distance[i] - distance[i-1];
    		if (i<(allValues)) {
    			if (acceleration[i-1] != 0) {
    				//the timeTaken array measures time COMPLETED whereas the velocity and acceleration array outputs FUTURE values (for the next increment)
    				//This is why we subtract i by 1 for velocity and acceleration
    				timeTaken[i] = (2*temp)/(clampVel[i-1]+clampVel[i]);
    			}
    			else {
    				//a reformat of d = v*t; there is no acceleration
    				timeTaken[i] = temp/clampVel[i];
    			}
    		}
    		else {
    			timeTaken[i] = (2*temp)/(clampVel[i-1]);
    		}
    	}
    	
    	//establishing return values
    	double returnDistance = 0;
    	double returnVelocity = 0;
    	double returnAccel = 0;
    	//These are for if you wanted to find stuff based on current coordinates, not really used
    	double currentDistance = 0;
    	double currentVelocity = 0;
    	double currentAccel = 0;
    	
		int currentIndex = 0;
		int l = 0;
		for (int i = 0; i < allValues; i++) {
			//finding where in the path the robot currently is based on current time
			//This is where the array function below is used
			if (time > arraySum(timeTaken, l)) {
				l = l+1;
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
		if (time > arraySum(timeTaken, timeTaken.length-1)) {
			currentIndex = timeTaken.length-1;
		}
		
		
		
		tempTestDistance[0]=0;
		for (int i=1; i<allValues; i++) {
			double timeDiff = timeTaken[i];
			tempTestDistance[i] = (1.0/2.0)*acceleration[i-1]*Math.pow(timeDiff, 2)+clampVel[i-1]*timeDiff;
		}
		for (int i=0; i<allValues; i++) {
			testDistance[i] = arraySum(tempTestDistance, i);
			if (i==(allValues-1)) {
				//System.out.println(Math.abs(distance[distance.length-1]-testDistance[i]) + " and " + testDistance[i]);
			}
			
		}
		
		//calculating the different in time and distance (between current position and last increment)
		double timeDifference = time - arraySum(timeTaken, currentIndex);
		double distanceChange = Math.abs(clampVel[currentIndex]*timeDifference+(0.5)*acceleration[currentIndex]*Math.pow(timeDifference, 2));
		//System.out.println("distance change: " + maxVelocities[currentIndex] + "; second part: " + (0.5)*acceleration[currentIndex]*Math.pow(timeDifference, 2));
		double velocityChange = Math.abs(clampVel[currentIndex]*timeDifference+(0.5)*acceleration[currentIndex]*Math.pow(timeDifference, 2));
		//System.out.println("current index: " + currentIndex + "; current distance: " + distance[currentIndex] + "; tiem difference: " + timeDifference);
		//calculating total distance covered
		returnDistance = distance[currentIndex]+distanceChange;
		
		if (acceleration[currentIndex]!=0) {
			returnAccel = acceleration[currentIndex];
			//calculating predicted velocity that the robot is currently at based on change in distance covered
			double squared = 2*returnAccel*velocityChange+Math.pow(clampVel[currentIndex], 2);
			//recalculating velocity, just in case
			if (squared<0) {
				//Clamping
				returnVelocity = Clamp(Double.NEGATIVE_INFINITY, max_velocity, -1.0*Math.sqrt(-squared))-limit;
			}
			else {
				returnVelocity = Clamp(Double.NEGATIVE_INFINITY, max_velocity, (Math.sqrt(squared)))-limit;
			}
		}
		else {
			returnAccel = 0;
			//velocity should stay the same if acceleration is 0
			returnVelocity = clampVel[currentIndex]-limit;
		}
		
		//Returning values for a certain x,y pair
		for (int i=0; i<x_values.length;i++) {
		    if (x_values[i]==current_x && y_values[i] == current_y) {
		        currentDistance = distance[i];
		        currentVelocity = clampVel[i]-limit;
		        currentAccel = acceleration[i];
		    }
		}
		//stores what the distance covered, current velocity, and current acceleration values should be in an array
		//stuff that begins with 'return' is from time, stuff that begins with 'current' is for location
		double[] returnArray = {returnDistance, returnVelocity, returnAccel, arraySum(timeTaken, timeTaken.length-1), distanceChange, time-arraySum(timeTaken, currentIndex-1), arraySum(timeTaken, currentIndex-1)};
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