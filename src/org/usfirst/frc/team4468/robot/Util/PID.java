package org.usfirst.frc.team4468.robot.Util;

import edu.wpi.first.wpilibj.Timer;

public class PID {
    
    //Constants
	public double maxVelocity = 6;
    private double kP;
    private double kI;
    private double kD;
    private boolean perTolerance;
    private boolean absTolerance;
    private boolean fFactorSetAccel;
    private boolean setRangeO;
    private boolean setRangeI;
    private boolean disabled;
    private double fFactorA;
    private double the_percent;
    private double the_distance;
    private double minRangeO;
    private double maxRangeO;
    private double minRangeI;
    private double maxRangeI;
    private double target;
    
    
    //Used for calculating deltaE (A.R.C of the error)
    private double previousError = 0;
    private double previousTime = 0;
    private double previousMeasure = 0;
    private double previousVelocity = 0;
    private double errorSum = 0;
    
    /**
     * Constructor. Creates a new PID instance.
     * 
     * @param P The P tuning constant. Set as 0 to disable.
     * @param I The I tuning constant. Set as 0 to disable.
     * @param D The D tuning constant. Set as 0 to disable.
     */
    public PID(double P, double I, double D){
        kP = P;
        kI = I;
        kD = D;
    }
    
    /**
     * Sets the output range
     * 
     * @param min The minimum range
     * @param max The maximum range
     */
    public void setOutputRange(double min, double max) {
    	    setRangeO = true;
    	    minRangeO = min;
    	    maxRangeO = max;
    }
    
    /**
    * Sets the input range
    * 
    * @param min The minimum range
    * @param max The maximum range
    */
    public void setInputRange(double min, double max) {
        setRangeI = true;
    	    minRangeI = min;
    	    maxRangeI = max;
    }
    
    /**
     * Sets the percent tolerance.
     * 
     * @param percent The specified percent tolerance.
     */
    public void setPerTolerance(double percent) {
    	    perTolerance = true;
    	    the_percent = percent;
    }
    
    /**
     * Sets the absolute tolerance.
     * 
     * @param percent The specified encoder distance tolerance.
     */
    public void setAbsTolerance(double distance) {
    	    absTolerance = true;
    	    the_distance = distance;
    }
    
    public boolean onTarget(double measure) {
        if(perTolerance) {
            return Math.abs(measure - target) < Math.abs(target * (the_percent/ 100));
        } else if (absTolerance) {
            return Math.abs(measure - target) < target;
        } else {
            throw new IllegalArgumentException("Please set a tolerance");
        }
    }
    
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
    
    /**
     * Sets the target.
     * 
     * @param theTarget The specified distance to go.
     */
    public void setPoint(double theTarget) {
    	    if (setRangeI) {
    	        // Limit the input to specified ranges
    	        target = Clamp(minRangeI, maxRangeI, theTarget);
    	    } else {
    	        target = theTarget;
    	    }
    }
    
    /**
     * Sets the acceleration feed forward factor.
     * 
     * @param fff The acceleration feed forward factor.
     */
    public void feedForwardAccel(double accel) {
    	    fFactorSetAccel = true;
    	    fFactorA = accel;
    }
    
    /**
     * Disables the PID
     */
    public void disable() {
    	    disabled = true;
    }
    
    /**
     * Returns the specified setpoint
     * 
     * @return the setpoint
     */
    public double getSetpoint() {
    	    return target;
    }
    
    /**
     * Run a single PID calculation on the given inputs. This does not loop
     * itself and must be placed in a loop.
     * 
     * @param measure The current value
     * @return The summation of the P, I, and D operations. Generally used
     * as an output.
     */
    public double calculate(double measure){
    	    double output;
    	    double error;
    	    // Setting the tolerance
    	    if (perTolerance) {
    	        // Setting the percent tolerance
    	        double percentDistance = Math.abs((the_percent/100)*target);
    	        if (Math.abs(previousError)<=percentDistance) {
    	            error = 0;
    	        } else {
    	            error = target - measure;
            }
    	    } else if (absTolerance) {
    	        // Setting the absolute tolerance
    	        if (Math.abs(previousError)<=the_distance) {
    	            error = 0;
    	        } else {
    	            error = target - measure;
    	        }
    	    } else if (disabled) {
    	        // Stop moving
    	        error = 0;
    	    } else {
    	        // Setting the error without any tolerance
    	        error = target - measure;
    	    }
        
        // The operational values
        double proportional = 0;
        double integral = 0;
        double derivative = 0;
        errorSum += error;
        double deltaE = previousError-error;
        double deltaT = Timer.getFPGATimestamp() - previousTime;
        
        /**** P ****/
        proportional = error*kP;
        
        /**** I ****/
        integral = errorSum*kI;
        
        /**** D ****/
        derivative = (deltaE/deltaT)*kD;
        
        output = proportional + integral + derivative;
        double velocity = (measure-previousMeasure)/(deltaT);
        
        if (fFactorSetAccel) {
        	    double acceleration = (velocity-previousVelocity)/(deltaT);
        	    // Setting the output if the f factor (acceleration) is set
        	    output += (acceleration*fFactorA);
        }
        
        previousVelocity = velocity; // Set the previous velocity location for the next cycle
        previousMeasure = measure; // Set the previous measured location for the next cycle
        previousError = error; // Set the previous error for the next cycle
        previousTime = Timer.getFPGATimestamp(); // Set the beginning time for the time measured between each output
        // Return
        if (setRangeO) {
            // Limit to the range if range is set
        	    return Clamp(minRangeO, maxRangeO, output);
        } else {
        	    return output;
        }
        
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
    
