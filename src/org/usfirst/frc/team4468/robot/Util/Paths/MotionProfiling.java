package org.usfirst.frc.team4468.robot.Util.Paths;

public class MotionProfiling {
	 public double[] motionProfiling(double endDistance, double cruiseV, double a, double time, double cruiseRatio) {
	    	// The cruise distance is the total distance times the cruise ratio
	    	double v_cruise = Clamp(0, maxVelocity, cruiseV);
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
	 
}
