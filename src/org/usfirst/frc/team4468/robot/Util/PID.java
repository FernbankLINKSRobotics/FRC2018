package org.usfirst.frc.team4468.robot.Util;

import edu.wpi.first.wpilibj.Timer;

public class PID {
    
    //Constants
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
    private double previousError;
    private double previousTime;
    private double previousMeasure;
    private double previousVelocity;
    private double errorSum;
    
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
    
