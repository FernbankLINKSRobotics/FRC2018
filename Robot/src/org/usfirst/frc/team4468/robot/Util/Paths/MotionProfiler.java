package org.usfirst.frc.team4468.robot.Util.Paths;


/*THESE ARE THE COORDINATES
 * static double[] xvalues = {
			0,
			0.0254,
			0.0762,
			0.4572,
			0.80645,
			1.143,
			1.362075,
			1.520825,
			1.597025
	};
	static double[] yvalues = {
			0,
			0.4064,
			0.688975,
			1.057275,
			1.400175,
			1.755775,
			2.066925,
			2.5019,
			3.01625
	};
 */

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
	
	double previous_angle = 0;
	
	double robot_width = .6;
	
	
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
	public double[] getMaxVelocity(double x_current, double y_current, double x_curve, double y_curve, double x_end, double y_end, boolean first) {

		
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
		
		double[] vector_curve = {(x_curve-x), (y_curve-y)};
		double[] vector_current = {(x_current-x), (y_current-y)};
		double[] vector_end = {(x_end-x), (y_end-y)};
		double[] vector_horizontal = {5, 0};
		double angle_current;
		double angle_curve;
		double angle_end;
		double test2 = Math.acos((vector_curve[0]*vector_horizontal[0]+vector_curve[1]*vector_horizontal[1])/(Math.sqrt(Math.pow(vector_curve[0], 2)+Math.pow(vector_curve[1], 2))*Math.sqrt(Math.pow(vector_horizontal[0], 2)+Math.pow(vector_horizontal[1], 2))));
		double test1 = Math.acos((vector_current[0]*vector_horizontal[0]+vector_current[1]*vector_horizontal[1])/(Math.sqrt(Math.pow(vector_current[0], 2)+Math.pow(vector_current[1], 2))*Math.sqrt(Math.pow(vector_horizontal[0], 2)+Math.pow(vector_horizontal[1], 2))));
		double test3 = Math.acos((vector_end[0]*vector_horizontal[0]+vector_end[1]*vector_horizontal[1])/(Math.sqrt(Math.pow(vector_end[0], 2)+Math.pow(vector_end[1], 2))*Math.sqrt(Math.pow(vector_horizontal[0], 2)+Math.pow(vector_horizontal[1], 2))));
		if (test1>(Math.PI/2)) {
			//angle_current = Math.PI-test1;
			//System.out.println("here");
			angle_current = Math.PI-test1;
		} else {
			angle_current = test1;
		} if (test2>(Math.PI/2)) {
			angle_curve = Math.PI-test2;
			//System.out.println("here");
		} else {
			angle_curve = test2;
		} if (test3>(Math.PI/2)) {
			angle_end = Math.PI-test3;
			//System.out.println("here");
		} else {
			angle_end = test3;
		}
		double x_reduce_current = (robot_width/2.0)*Math.sin(angle_current);
		double y_reduce_current;
		if (Math.pow((robot_width/2.0), 2)-Math.pow(x_reduce_current, 2)<0) {
			y_reduce_current = -Math.sqrt(Math.abs(Math.pow((robot_width/2.0), 2)-Math.pow(x_reduce_current, 2)));
		} else {
			y_reduce_current = Math.sqrt(Math.pow((robot_width/2.0), 2)-Math.pow(x_reduce_current, 2));
		}
		double x_reduce_curve = (robot_width/2.0)*Math.sin(angle_curve);
		double y_reduce_curve;
		if (Math.pow((robot_width/2.0), 2)-Math.pow(x_reduce_curve, 2)<0) {
			y_reduce_curve = -Math.sqrt(Math.abs(Math.pow((robot_width/2.0), 2)-Math.pow(x_reduce_curve, 2)));
		} else {
			y_reduce_curve = Math.sqrt(Math.pow((robot_width/2.0), 2)-Math.pow(x_reduce_curve, 2));
		}
		double x_reduce_end = (robot_width/2.0)*Math.sin(angle_end);
		double y_reduce_end;
		if (Math.pow((robot_width/2.0), 2)-Math.pow(x_reduce_end, 2)<0) {
			y_reduce_end = -Math.sqrt(Math.abs(Math.pow((robot_width/2.0), 2)-Math.pow(x_reduce_end, 2)));
		} else {
			y_reduce_end = Math.sqrt(Math.pow((robot_width/2.0), 2)-Math.pow(x_reduce_end, 2));
		}
		double ah;
		if (previous_angle==0) {
			ah = angle_current;
		}
		ah = Math.abs(angle_current-previous_angle);
		boolean add_left = false;
		boolean add_right=false;
		double ax_rt;
		double ay_rt;
		double bx_rt;
		double by_rt;
		double cx_rt;
		double cy_rt;
		double ax_lf;
		double ay_lf;
		double bx_lf;
		double by_lf;
		double cx_lf;
		double cy_lf;
		//y inc and x inc
		if (y_end>y_current && x_end>x_current) {
			//subtract x, add y for left
			ax_lf = ax-x_reduce_current;
			ay_lf = ay+y_reduce_current;
			bx_lf = bx-x_reduce_curve;
			by_lf = by+y_reduce_curve;
			cx_lf = cx-x_reduce_end;
			cy_lf = cy+y_reduce_end;
			
			//opposite for right
			ax_rt = ax+x_reduce_current;
			ay_rt = ay-y_reduce_current;
			bx_rt = bx+x_reduce_curve;
			by_rt = by-y_reduce_curve;
			cx_rt = cx+x_reduce_end;
			cy_rt = cy-y_reduce_end;
		}
		//y inc and x dec
		else if (y_end>y_current && x_current>x_end) {
			ax_lf = ax-x_reduce_current;
			ay_lf = ay-y_reduce_current;
			bx_lf = bx-x_reduce_curve;
			by_lf = by-y_reduce_curve;
			cx_lf = cx-x_reduce_end;
			cy_lf = cy-y_reduce_end;
			
			//opposite for right
			ax_rt = ax+x_reduce_current;
			ay_rt = ay+y_reduce_current;
			bx_rt = bx+x_reduce_curve;
			by_rt = by+y_reduce_curve;
			cx_rt = cx+x_reduce_end;
			cy_rt = cy+y_reduce_end;
		}
		//y dec and x inc
		else if (y_current>y_end && x_end>x_current) {
			ax_lf = ax+x_reduce_current;
			ay_lf = ay+y_reduce_current;
			bx_lf = bx+x_reduce_curve;
			by_lf = by+y_reduce_curve;
			cx_lf = cx+x_reduce_end;
			cy_lf = cy+y_reduce_end;
			
			//opposite for right
			ax_rt = ax-x_reduce_current;
			ay_rt = ay-y_reduce_current;
			bx_rt = bx-x_reduce_curve;
			by_rt = by-y_reduce_curve;
			cx_rt = cx-x_reduce_end;
			cy_rt = cy-y_reduce_end;
		}
		else if (y_current>y_end && x_current>x_end) {
			ax_lf = ax+x_reduce_current;
			ay_lf = ay-y_reduce_current;
			bx_lf = bx+x_reduce_curve;
			by_lf = by-y_reduce_curve;
			cx_lf = cx+x_reduce_end;
			cy_lf = cy-y_reduce_end;
			
			//opposite for right
			ax_rt = ax-x_reduce_current;
			ay_rt = ay+y_reduce_current;
			bx_rt = bx-x_reduce_curve;
			by_rt = by+y_reduce_curve;
			cx_rt = cx-x_reduce_end;
			cy_rt = cy+y_reduce_end;
		}
		else {
			System.out.println("change coordinates");
			ax_lf = ax;
			ay_lf = ay;
			bx_lf = bx;
			by_lf = by;
			cx_lf = cx;
			cy_lf = cy;
			//opposite for right
			ax_rt = ax;
			ay_rt = ay;
			bx_rt = bx;
			by_rt = by;
			cx_rt = cx;
			cy_rt = cy;
		}
		if ((ay_rt-ay_lf)/(ax_rt-ax_lf)<0) {
			add_left = true;
		}
		else {
			add_right = true;
		}
		double distance_right = form_circle(ax_rt, ay_rt, bx_rt, by_rt, cx_rt, cy_rt, ah, x, y)[0];
		//System.out.println(ax_lf + ", " + ay_lf + ", " + bx_lf + ", " + by_lf + ", " + cx_lf + ", " + cy_lf);
		double maxVelocity_right = form_circle(ax_rt, ay_rt, bx_rt, by_rt, cx_rt, cy_rt, ah, x, y)[1];
		double distance_left = form_circle(ax_lf, ay_lf, bx_lf, by_lf, cx_lf, cy_lf, ah, x,y)[0];
		double maxVelocity_left = form_circle(ax_lf, ay_lf, bx_lf, by_lf, cx_lf, cy_lf, ah,x,y)[1];
		if (first) {
			if (add_left) {
				distance_left = distance_left+form_circle(ax_lf, ay_lf, bx_lf, by_lf, cx_lf, cy_lf, ah,x,y)[2];
				distance_right = distance_right-form_circle(ax_rt, ay_rt, bx_rt, by_rt, cx_rt, cy_rt, ah,x,y)[2];

			}
			if (add_right) {
				distance_right = distance_right+form_circle(ax_rt, ay_rt, bx_rt, by_rt, cx_rt, cy_rt, ah,x,y)[2];
				distance_left = distance_left-form_circle(ax_lf, ay_lf, bx_lf, by_lf, cx_lf, cy_lf, ah,x,y)[2];
			}
		}
		double[] returnArray = {maxVelocity_right, maxVelocity_left, distance_right, distance_left};
		//System.out.println(bx_lf + ", " + bx + ", " + bx_rt+ ", " + by_lf + ", " +by+ ", " +by_rt);
		previous_angle = angle_end;
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
		double[] actualmax_left = new double[x_values.length+1];
		double[] actualmax_right = new double[x_values.length+1];
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
				if (i==0) {
					actualmax_left[i] = Clamp(0, max_velocity, (getMaxVelocity(x_values[i], y_values[i], x_values[i+1], y_values[i+1], x_values[i+2], y_values[i+2], true)[1]-limit));
					actualmax_right[i] = Clamp(0, max_velocity, (getMaxVelocity(x_values[i], y_values[i], x_values[i+1], y_values[i+1], x_values[i+2], y_values[i+2], true)[0]-limit));
					tempDistance_left[i] = (getMaxVelocity(x_values[i], y_values[i], x_values[i+1], y_values[i+1], x_values[i+2], y_values[i+2], true)[3])/2;
					tempDistance_right[i] = (getMaxVelocity(x_values[i], y_values[i], x_values[i+1], y_values[i+1], x_values[i+2], y_values[i+2], true)[2])/2;
					actualmax_left[i+1] = Clamp(0, max_velocity, (getMaxVelocity(x_values[i], y_values[i], x_values[i+1], y_values[i+1], x_values[i+2], y_values[i+2], true)[1]-limit));
					actualmax_right[i+1] = Clamp(0, max_velocity, (getMaxVelocity(x_values[i], y_values[i], x_values[i+1], y_values[i+1], x_values[i+2], y_values[i+2], true)[0]-limit));
					//Shows distance that WAS accomplished, why we add +1 to i
					tempDistance_left[i+1] = (getMaxVelocity(x_values[i], y_values[i], x_values[i+1], y_values[i+1], x_values[i+2], y_values[i+2], true)[3])/2;
					tempDistance_right[i+1] = (getMaxVelocity(x_values[i], y_values[i], x_values[i+1], y_values[i+1], x_values[i+2], y_values[i+2], true)[2])/2;
				}
				else if (i>0 && i < allValues-1) {
					actualmax_left[i+1] = Clamp(0, max_velocity, (getMaxVelocity(x_values[i], y_values[i], x_values[i+1], y_values[i+1], x_values[i+2], y_values[i+2], false)[1]-limit));
					actualmax_right[i+1] = Clamp(0, max_velocity, (getMaxVelocity(x_values[i], y_values[i], x_values[i+1], y_values[i+1], x_values[i+2], y_values[i+2], false)[0]-limit));
					//Shows distance that WAS accomplished, why we add +1 to i
					tempDistance_left[i+1] = (getMaxVelocity(x_values[i], y_values[i], x_values[i+1], y_values[i+1], x_values[i+2], y_values[i+2], false)[3])/2;
					tempDistance_right[i+1] = (getMaxVelocity(x_values[i], y_values[i], x_values[i+1], y_values[i+1], x_values[i+2], y_values[i+2], false)[2])/2;
					if (i < (allValues-2)) {
						actualmax_left[i] = Clamp(0, max_velocity, (getMaxVelocity(x_values[i], y_values[i], x_values[i+1], y_values[i+1], x_values[i+2], y_values[i+2], false)[1]-limit));
						actualmax_right[i] = Clamp(0, max_velocity, (getMaxVelocity(x_values[i], y_values[i], x_values[i+1], y_values[i+1], x_values[i+2], y_values[i+2], false)[0]-limit));
						tempDistance_left[i] = (getMaxVelocity(x_values[i], y_values[i], x_values[i+1], y_values[i+1], x_values[i+2], y_values[i+2], false)[3])/2;
						tempDistance_right[i] = (getMaxVelocity(x_values[i], y_values[i], x_values[i+1], y_values[i+1], x_values[i+2], y_values[i+2], false)[2])/2;
					}
					//Use if statements so we don't get an error about array size
					else {
						actualmax_left[i] = Clamp(0, max_velocity, getMaxVelocity(x_values[i-2], y_values[i-2], x_values[i-1], y_values[i-1], x_values[i], y_values[i], false)[1]);
						actualmax_right[i] = Clamp(0, max_velocity, getMaxVelocity(x_values[i-2], y_values[i-2], x_values[i-1], y_values[i-1], x_values[i], y_values[i], false)[0]);
						tempDistance_left[i] = (getMaxVelocity(x_values[i-2], y_values[i-2], x_values[i-1], y_values[i-1], x_values[i], y_values[i], false)[3])/2;
						tempDistance_right[i] = (getMaxVelocity(x_values[i-2], y_values[i-2], x_values[i-1], y_values[i-1], x_values[i], y_values[i], false)[2])/2;
					}
				}
				else {
					actualmax_left[i] = Clamp(0, max_velocity, getMaxVelocity(x_values[i-2], y_values[i-2], x_values[i-1], y_values[i-1], x_values[i], y_values[i], false)[1]);
					actualmax_right[i] = Clamp(0, max_velocity, getMaxVelocity(x_values[i-2], y_values[i-2], x_values[i-1], y_values[i-1], x_values[i], y_values[i], false)[0]);
				}
				actualmax_left[allValues]=0;
				actualmax_right[allValues]=0;
				tempDistance_left[allValues-1] = (getMaxVelocity(x_values[allValues-3], y_values[allValues-3], x_values[allValues-2], y_values[allValues-2], x_values[allValues-1], y_values[allValues-1], false)[3])/2;
				tempDistance_right[allValues-1] = (getMaxVelocity(x_values[allValues-3], y_values[allValues-3], x_values[allValues-2], y_values[allValues-2], x_values[allValues-1], y_values[allValues-1], false)[2])/2;
				
		}
		double[] uh_left = new double[allValues];
		double[] rut_right = new double[allValues];
		uh_left[0]=0;
		rut_right[0]=0;
        for (int i=1; i<allValues;i++) {
        	uh_left[i] = tempDistance_left[i-1];
        	rut_right[i] = tempDistance_right[i-1];
        }
		//Making the distance array cumulative
		for (int i = 0; i< tempDistance_left.length;i++) {
			distance_left[i] = arraySum(uh_left, i);
			distance_right[i] = arraySum(rut_right, i);
		}
		
		//Sets acceleration depending on if two velocities are different
		//Acceleration period will most likely happen once every three points
		for (int i = 1; i < x_values.length; i++) {
			if (actualmax_left[i-1] != actualmax_left[i]) {
				double tdistance = distance_left[i]-distance_left[i-1];
				acceleration_left[i-1] = Clamp(Double.NEGATIVE_INFINITY, max_acceleration, (Math.pow(actualmax_left[i], 2)-Math.pow(actualmax_left[i-1], 2))/(2*tdistance));
				//acceleration is set for the current to the next point
			}
			if (actualmax_right[i-1] != actualmax_right[i]) {
				double tdistance = distance_right[i]-distance_right[i-1];
				acceleration_right[i-1] = Clamp(Double.NEGATIVE_INFINITY, max_acceleration, (Math.pow(actualmax_right[i], 2)-Math.pow(actualmax_right[i-1], 2))/(2*tdistance));
				//acceleration is set for the current to the next point
			}
		}
		
		//the start distance and max velocity will be zero
		
		
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
			if (v_left < actualmax_left[y]) {
				y = y + 1;
			}
			else {
				//Subtracting one because we don't want the accelerated velocity to be OVER max velocity
				if (y<=((allValues+1)/2.0)) {
					accelIndex_left = y-1;
				} else {
					accelIndex_left = (int)(allValues/2.0);
				}
			}
			if (v_right < actualmax_right[z]) {
				z = z + 1;
			}
			else {
				//Subtracting one because we don't want the accelerated velocity to be OVER max velocity
				if (z<=((allValues+1)/2.0)) {
					accelIndex_right = z-1;
				} else {
					accelIndex_right = (int)(allValues/2.0);
				}
			}
		}

		if (accelIndex_left==0 || accelIndex_right==0) {
			//System.out.println(maxVelocities_left[1] + ", " + (Math.sqrt(2*ta*distance_left[1])));
			System.out.println("lower distance between points or lower acceleration");
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
		maxVelocities_right[0]=0;
		maxVelocities_left[0]=0;
		double max_left = Math.sqrt(2*acceleration_left[0]*(distance_left[accelIndex_left]));
		double max_right = Math.sqrt(2*acceleration_right[0]*(distance_right[accelIndex_right]));
		
    	
    	//SETTING DECELERATION
    	//We want this to be an overapproximation instead of an underapproximation so it fully goes to 0
		
    	//double testing_left = (-Math.pow(max_left, 2))/(2*(distance_left[allValues-1]-distance_left[accelIndex_left]));
    	double yellow_left=.1;
    	boolean scuse = false;
    	double[] test_accel_left = new double[allValues];
    		for (int m=0; m<1000; m++) {
    			for (int i=0; i<(accelIndex_left);i++) {
    				if (!scuse) {
    					test_accel_left[i] = acceleration_left[i]-yellow_left;
    				}
    			}
    			double max_left_test = Math.sqrt(2*test_accel_left[0]*(distance_left[accelIndex_left]));
    			double decel_test = (-Math.pow(max_left_test, 2))/(2*(distance_left[allValues-1]-distance_left[accelIndex_left+1]));
    			
    			if (-decel_test>max_acceleration) {
    				yellow_left = yellow_left+.01;
    			}
    			else {
    				scuse = true;
    				for (int h=0; h<(accelIndex_left); h++) {
    					acceleration_left[h] = test_accel_left[0];
    				} for (int h=accelIndex_left+1; h<allValues;h++) {
    					acceleration_left[h] = decel_test;
    				}
    				acceleration_left[accelIndex_left] = 0;
    			}
    	}
    	//double testing_right = (-Math.pow(max_right, 2))/(2*(distance_right[allValues-1]-distance_right[accelIndex_right]));
    	double yellow_right=0;
    	boolean yum = false;
    	double[] test_accel_right = new double[allValues];

    		for (int m=0; m<1000; m++) {
    			for (int i=0; i<(accelIndex_right);i++) {
    				if (!yum) {
    					test_accel_right[i] = acceleration_right[i]-yellow_right;
    				}
    			}
    			double max_right_test = Math.sqrt(2*test_accel_right[0]*(distance_right[accelIndex_right]));
    			double decel_test = (-Math.pow(max_right_test, 2))/(2*(distance_right[allValues-1]-distance_right[accelIndex_right]+1));
    			//double distance_test = (Math.pow(clampVel_right[accelIndex_right-1], 2))/(2*test_accel_right[0]);
    			if (-decel_test>max_acceleration) {
    				yellow_right = yellow_right+.01;
    			}
    			
    			else {
    				yum= true;
    				for (int h=0; h<(accelIndex_right); h++) {
    					acceleration_right[h] = test_accel_right[h];
    				} for (int h=accelIndex_right+1; h<allValues;h++) {
    					acceleration_right[h] = decel_test;
    				}
    				acceleration_right[accelIndex_right] = 0;
    			}
    		
    	}
    	for (int i=0; i<allValues; i++) {
    		//System.out.println(acceleration_right[i]);
    	}
    	
    	
    	maxVelocities_left[0]=0;
    	maxVelocities_right[0]=0;
    	for (int i=1; i<allValues;i++) {
    		maxVelocities_left[i] = Math.sqrt(Math.pow(maxVelocities_left[i-1], 2)+2*acceleration_left[i-1]*(distance_left[i]-distance_left[i-1]));
    		
    	}
    	
    	for (int i = 1; i < (allValues); i++) {
    		maxVelocities_left[i] = Clamp(Double.NEGATIVE_INFINITY, max_left, Math.sqrt(Math.pow(maxVelocities_left[i-1], 2)+2*acceleration_left[i-1]*(distance_left[i]-distance_left[i-1])));
    		
    	}
    	for (int i =1; i < (allValues); i++) {
    		maxVelocities_right[i] = Clamp(Double.NEGATIVE_INFINITY, max_right, Math.sqrt(Math.pow(maxVelocities_right[i-1], 2)+2*acceleration_right[i-1]*(distance_right[i]-distance_right[i-1])));
    	}
    	if (maxVelocities_right[allValues-1]<0) {
    		//do nothing
    	} else {
    		acceleration_right[allValues-1] = (-Math.pow(maxVelocities_right[allValues-2], 2))/(2*(distance_right[allValues-1]-distance_right[allValues-2]));
    		maxVelocities_right[allValues-1] = 0;
    	}
    	if (maxVelocities_left[allValues-1]<0) {
    		//do nothing
    	} else {
    		acceleration_left[allValues-1] = (-Math.pow(maxVelocities_left[allValues-2], 2))/(2*(distance_left[allValues-1]-distance_left[allValues-2]));
    		maxVelocities_left[allValues-1] = 0;
    	}
    	
    	 
    	
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
    			} else {
    				//a reformat of d = v*t; there is no acceleration
    				timeTaken_left[i] = temp_left/clampVel_left[i];
    			} if (acceleration_right[i-1] != 0) {
    				//the timeTaken array measures time COMPLETED whereas the velocity and acceleration array outputs FUTURE values (for the next increment)
    				//This is why we subtract i by 1 for velocity and acceleration
    				timeTaken_right[i] = (2*temp_right)/(clampVel_right[i-1]+clampVel_right[i]);
    			} else {
    				//a reformat of d = v*t; there is no acceleration
    				timeTaken_right[i] = temp_right/clampVel_right[i];
    			}
    		}
    		else {
    			timeTaken_left[i] = (2*temp_left)/(clampVel_left[i-1]);
    			timeTaken_right[i] = (2*temp_right)/(clampVel_right[i-1]);
    	
    		}
    	}
    	
    	
    	double[] velocitiesthree_right = new double[allValues];
    	double[] velocitiesthree_left = new double[allValues];
    	double[] accelerationthree_left=new double[allValues];
    	double[] accelerationthree_right=new double[allValues];
    	double[] newTime=new double[allValues];
    	//COME BACK TO THIS SECTION LATER
    	for (int i=1; i<allValues; i++) {
    		if (timeTaken_left[i]>timeTaken_right[i]) {
    			double tempTime = timeTaken_left[i];
    			accelerationthree_left[i] = (maxVelocities_left[i]-maxVelocities_left[i-1])/tempTime;
    			accelerationthree_right[i] = (maxVelocities_right[i]-maxVelocities_right[i-1])/tempTime;
    			if (Math.abs(accelerationthree_left[i])>max_acceleration||Math.abs(accelerationthree_right[i])>max_acceleration) {
    				newTime[i] = tempTime+.1;
    				for (int g=0; g<100; g++) {
    					accelerationthree_left[i] = (maxVelocities_left[i]-maxVelocities_left[i-1])/newTime[i];
    		    		accelerationthree_right[i] = (maxVelocities_right[i]-maxVelocities_right[i-1])/newTime[i];
    		    		if (Math.abs(accelerationthree_left[i])>max_acceleration||Math.abs(accelerationthree_right[i])>max_acceleration) {
    		    			newTime[i] = newTime[i]+.1;
    		    		}
    				}
    			}
    			else {
    				newTime[i] = tempTime;
    			}
    			
    			
	    	}
	    	else {
	    		double tempTime = timeTaken_right[i];
	    		accelerationthree_left[i] = (maxVelocities_left[i]-maxVelocities_left[i-1])/tempTime;
    			accelerationthree_right[i] = (maxVelocities_right[i]-maxVelocities_right[i-1])/tempTime;
    			if (Math.abs(accelerationthree_left[i])>max_acceleration||Math.abs(accelerationthree_right[i])>max_acceleration) {
    				newTime[i] = tempTime+.1;
    				for (int g=0; g<100; g++) {
    					accelerationthree_left[i] = (maxVelocities_left[i]-maxVelocities_left[i-1])/newTime[i];
    		    		accelerationthree_right[i] = (maxVelocities_right[i]-maxVelocities_right[i-1])/newTime[i];
    		    		if (Math.abs(accelerationthree_left[i])>max_acceleration||Math.abs(accelerationthree_right[i])>max_acceleration) {
    		    			newTime[i] = newTime[i]+.1;
    		    		}
    				}
    			}
    			else {
    				newTime[i] = tempTime;
    			}
	    		
	    	}
    	}
    	
    	double[] test_vel_left = new double[allValues];
    	double[] test_vel_right = new double[allValues];
    	test_vel_left[0]=0;
    	test_vel_right[0]=0;
    	
    	velocitiesthree_left[0] = 0;
    	velocitiesthree_right[0] = 0;
    	velocitiesthree_left[allValues-1] = 0;
    	velocitiesthree_right[allValues-1] = 0;
    	
    	for (int k=0; k<(allValues-1);k++) {
			double tempTime = newTime[k+1];
			//accelerationthree_left[k] = (2.0*(tempDistance-velocitiesthree_left[k]*tempTime))/Math.pow(tempTime, 2);
			//accelerationthree_left[k] = (Math.pow(maxVelocities_left[k+1], 2)-Math.pow(maxVelocities_left[k], 2))/(2*(distance_left[k+1]-distance_left[k]));
			accelerationthree_left[k] = (maxVelocities_left[k+1]-maxVelocities_left[k])/tempTime;
			//System.out.println(accelerationthree_left[k] + ", " + k);
		}
    	for (int i=1; i<(allValues);i++) {
    		double tempTime = newTime[i];
    		velocitiesthree_left[i] = velocitiesthree_left[i-1]+accelerationthree_left[i-1]*tempTime;
    		//velocitiesthree_left[i] = (tempDistance/tempTime)-(accelerationthree_left[i]*tempTime)/(2);
    		
    	}
    	
    	for (int k=0; k<(allValues-1);k++) {
			double tempTime = newTime[k+1];
			accelerationthree_right[k] = (maxVelocities_right[k+1]-maxVelocities_right[k])/tempTime;
		
    	}
    	for (int i=1; i<(allValues);i++) {
    		double tempTime = newTime[i];
    		velocitiesthree_right[i] = velocitiesthree_right[i-1]+accelerationthree_right[i-1]*tempTime;
    		
    		//System.out.println(velocitiesthree_left[i]);
    	}
    	
    	
    	double[] temporary_left = new double[allValues];
    	double[] temporary_right = new double[allValues];
    	for (int i=1; i<allValues;i++) {
    		double tempTime = newTime[i];
    		temporary_left[i] = (.5)*accelerationthree_left[i-1]*Math.pow(tempTime, 2)+velocitiesthree_left[i-1]*tempTime;
    		temporary_right[i] = (.5)*accelerationthree_right[i-1]*Math.pow(tempTime, 2)+velocitiesthree_right[i-1]*tempTime;
    		
    	}
    	double error_left = Math.abs(distance_left[allValues-1]-arraySum(temporary_left, allValues-1));
    	double error_right = Math.abs(distance_right[allValues-1]-arraySum(temporary_right, allValues-1));
    	//System.out.println((error_left/arraySum(temporary_left, allValues-1))*100 + ", " + (error_right/arraySum(temporary_right, allValues-1))*100);
    	double[] new_distance_left = new double[allValues];
    	double[] new_distance_right = new double[allValues];
    	for (int i=0; i<allValues;i++) {
    		//System.out.println(temporary_right[i] + ": " + Math.abs(rut_right[i]-temporary_right[i]) + ", " + x_values[i]);
    		new_distance_left[i] = arraySum(temporary_left, i);
    		new_distance_right[i] = arraySum(temporary_right, i);
    	}
		double[] newVelocities_left = new double[allValues];
		double[] newVelocities_right = new double[allValues];
		double[] newAcceleration_left = new double[allValues];
		double[] newAcceleration_right = new double[allValues];

    		for (int i=0; i<(allValues);i++) {
    			if (i<(allValues-1)) {
    				newAcceleration_left[i] = accelerationthree_left[i];
    				newAcceleration_right[i] = accelerationthree_right[i];
    			}
    			newVelocities_left[i] = velocitiesthree_left[i];
    			newVelocities_right[i] = velocitiesthree_right[i];
    			
    		}

    	//System.out.println(arraySum(timeTaken_left, timeTaken_left.length-1));
    	//establishing return values
    	double returnDistance_left = 0;
    	double returnVelocity_left = 0;
    	double returnAccel_left = 0;
    	double returnDistance_right = 0;
    	double returnVelocity_right = 0;
    	double returnAccel_right = 0;
    	

		int l=0;
		int currentIndex = 0;
		for (int i = 0; i < allValues; i++) {
			//finding where in the path the robot currently is based on current time
			//This is where the array function below is used
			if (time > arraySum(newTime, l)) {
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
		if (time > arraySum(newTime, newTime.length-1)) {
			currentIndex = newTime.length-1;
		}
	
		
		
		double tempAccel_left = newAcceleration_left[currentIndex];
		double tempVel_left = newVelocities_left[currentIndex];
		double tempAccel_right = newAcceleration_right[currentIndex];
		double tempVel_right = newVelocities_right[currentIndex];
		

		
		//LEFT
		//calculating the different in time and distance (between current position and last increment)
		double timeDifference_left = time - arraySum(newTime, currentIndex);
		double distanceChange_left = Math.abs(tempVel_left*timeDifference_left+(0.5)*tempAccel_left*Math.pow(timeDifference_left, 2));
		//calculating total distance covered
		returnDistance_left = new_distance_left[currentIndex]+distanceChange_left;
		//System.out.println(time + ", " + tempVel_left + ", " + tempAccel_left);

		//RIGHT
		double timeDifference_right = time - arraySum(newTime, currentIndex);
		double distanceChange_right = Math.abs(tempVel_right*timeDifference_right+(0.5)*tempAccel_right*Math.pow(timeDifference_right, 2));
		returnDistance_right = new_distance_right[currentIndex]+distanceChange_right;
		
		//LEFT
		if (tempAccel_left!=0.0) {
			returnAccel_left = newAcceleration_left[currentIndex];
			//calculating predicted velocity that the robot is currently at based on change in distance covered
			double squared = 2*returnAccel_left*distanceChange_left+Math.pow(tempVel_left, 2);
			//recalculating velocity, just in case
			if (squared<0) {
				//Clamping
				returnVelocity_left = Clamp(Double.NEGATIVE_INFINITY, max_left, -1.0*Math.sqrt(-squared))-limit;
			}
			else {
				returnVelocity_left = Clamp(Double.NEGATIVE_INFINITY, max_left, (Math.sqrt(squared)))-limit;
				//System.out.println(max_left);
			}
		}
		else {
			returnAccel_left = 0.0;
			//velocity should stay the same if acceleration is 0
			if (newVelocities_left[currentIndex]>limit) {
				returnVelocity_left = tempVel_left-limit;
			}
			else {
				returnVelocity_left = tempVel_left;
			}
		}
		//RIGHT
		if (tempAccel_right!=0.0) {
			returnAccel_right = newAcceleration_right[currentIndex];
			//calculating predicted velocity that the robot is currently at based on change in distance covered
			double squared = 2*returnAccel_right*distanceChange_right+Math.pow(tempVel_right, 2);
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
			if (newVelocities_right[currentIndex]>limit) {
				returnVelocity_right = tempVel_right-limit;
			}
			else {
				returnVelocity_right = tempVel_right;
			}
		}
		
		
		//stores what the distance covered, current velocity, and current acceleration values should be in an array
		//stuff that begins with 'return' is from time, stuff that begins with 'current' is for location
		double[] returnArray = {returnDistance_left, returnVelocity_left, returnAccel_left, arraySum(newTime, newTime.length-1), returnDistance_right, returnVelocity_right, returnAccel_right, arraySum(timeTaken_right, timeTaken_right.length-1)};
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
	public double[] getVelocity(double currentDist_left, double increment) {
		//Getting max time to get to the end
		double kale = execute2D(0.0, 0.0, 0.0)[3];
		double[] kaleDistance_left = new double[(int)(kale*100)+1];
		double[] kaleVelocity_left = new double[(int)(kale*100)+1];
		double[] kaleAcceleration_left = new double[(int)(kale*100)+1];
		double[] kaleDistance_right = new double[(int)(kale*100)+1];
		double[] kaleVelocity_right = new double[(int)(kale*100)+1];
		double[] kaleAcceleration_right = new double[(int)(kale*100)+1];
		//going through time to create some sort of a timeline
		for (double i=0; i<kale;i=i+.01) {
			//Adding it to an array
			kaleDistance_left[(int) (i*100)] = execute2D(i, 0.0, 0.0)[0];
			kaleVelocity_left[(int) (i*100)] = execute2D(i, 0.0, 0.0)[1];
			kaleAcceleration_left[(int) (i*100)] = execute2D(i, 0.0, 0.0)[2];
			kaleDistance_right[(int) (i*100)] = execute2D(i, 0.0, 0.0)[4];
			kaleVelocity_right[(int) (i*100)] = execute2D(i, 0.0, 0.0)[5];
			kaleAcceleration_right[(int) (i*100)] = execute2D(i, 0.0, 0.0)[6];
		}
		int l =0;
		int currentIndex=0;
		//Finding where on the timeline you are based on current distance
		//utilizing time to find velocity and acceleration
		for (int i = 0; i < kaleDistance_left.length; i++) {
			if (currentDist_left > kaleDistance_left[l]) {
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
		double[] returnArray = {kaleDistance_left[currentIndex+(int)(increment*100)], kaleDistance_right[currentIndex+(int)(increment*100)]};
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

    
    public double[] form_circle(double ax, double ay, double bx, double by, double cx, double cy, double prev_angle, double center_x, double center_y) {
    	
    	double x=center_x;
    	double y=center_y;
		
		double radius = Math.hypot((ax-x), (ay-y));
		double chord_length_one = Math.hypot((cx-bx), (cy-by));
		double chord_length_two = Math.hypot((bx-ax), (by-ay));
		
		double angle_one = Math.acos(((2*Math.pow(radius, 2))-Math.pow(chord_length_one, 2))/(2*Math.pow(radius, 2)));
		double angle_two = Math.acos(((2*Math.pow(radius, 2))-Math.pow(chord_length_two, 2))/(2*Math.pow(radius, 2)));
		double distance_one = angle_one*radius;
		double distance_two = angle_two*radius;
		double distance = distance_one+distance_two;
		
		double add_distance = prev_angle*radius;
		
		double maxVelocity = Math.sqrt(radius*9.81);
		//System.out.println(radius + ": " + ax + ", " + ay + ", " + bx + ", " + by + ", " + cx + ", "+ cy);
		double[] returnArray = {distance, maxVelocity, add_distance};
		return returnArray;
    }
	 
    
 
}

