package org.usfirst.frc.team4468.robot.Util;

import edu.wpi.first.wpilibj.Timer;

/**
 * 
 * This class is designed to run a PID loop on any input. This class is
 * designed to be capable of running PID wherever necessary in the event
 * that PID is not defaultly supported by the hardware. PID is designed to
 * calculate error (how far away you are from your target value) and lower
 * the output as you approach your target value so that you avoid
 * overshooting. PID stands for proportional, integral, and derivative.
 * Three scaling constants are given for each operation to tune the outputs.
 * The outputs of these functions are summed; this is the returned value.
 * 
 * @author Toaster Tech
 * @link https://github.com/ToasterTechFRC/Toaster-Base-Bot/blob/master/Toaster%20Base%20Bot/src/utils/PID.java
 *
 */
public class PID {
    
    //Constants
    private double kP;
    private double kI;
    private double kD;
    private boolean perTolerance;
    private double the_percent;
    private boolean absTolerance;
    private double the_distance;
    
    //Used for calculating deltaE (A.R.C of the error)
    private double previousError;
    private double previousTime;
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
    
    /**
     * Run a single PID calculation on the given inputs. This does not loop
     * itself and must be placed in a loop.
     * 
     * @param target The desired value
     * @param measure The current value
     * @return The summation of the P, I, and D operations. Generally used
     * as an output.
     */
    public double calculate(double target, double measure){
    	
    	double error;
    	// Setting the tolerance
    	if (perTolerance) {
    		// Setting the percent tolerance
    		double percentDistance;
    		percentDistance = the_percent*target;
    		if (Math.abs(previousError)<=percentDistance) {
            	error = 0;
            }
            else {
            	error = target - measure;
            }
    	}
    	else if (absTolerance) {
    		// Setting the absolute tolerance
    		if (Math.abs(previousError)<=the_distance) {
    			error = 0;
    		}
    		else {
    			error = target - measure;
    		}
    	}
    	else {
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
        previousError = error; // Set the previous error for the next cycle
        previousTime = Timer.getFPGATimestamp(); // Set the beginning time for the time measured between each output
        return (proportional + integral + derivative);
        // Return
        
    }
    
}
    
