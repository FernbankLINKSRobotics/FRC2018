package org.usfirst.frc.team4468.robot.Util;

import edu.wpi.first.wpilibj.Timer;

public class PID {
    
    //Constants
    private double kP = 0;
    private double kI = 0;
    private double kD = 0;
    private double tolerance = 0;
    private boolean rangeIn = false;
    private boolean rangeOut = false;
    private boolean continuous = false;
    private double maxIn = 0;
    private double minIn = 0;
    private double maxOut = 0;
    private double minOut = 0;
    private double target = 0;
    
    
    //Used for calculating deltaE (A.R.C of the error)
    private double prevErr = Double.MAX_VALUE;
    private double prevMeasure = 0;
    private double totalError = 0;
    private double prevTime = 0;
    private double err = 0;
    
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
        prevTime = Timer.getFPGATimestamp();
    }
    
    /**
     * Constructor. Creates a new PID instance.
     * 
     * @param P The P tuning constant. Set as 0 to disable.
     * @param I The I tuning constant. Set as 0 to disable.
     * @param D The D tuning constant. Set as 0 to disable.
     * @param T the absolute tolerance of the system.
     */
    public PID(double P, double I, double D, double T){
        kP = P;
        kI = I;
        kD = D;
        tolerance = T;
        prevTime = Timer.getFPGATimestamp();
    }
    
    /**
     * Constructor. Creates a new PID instance.
     * 
     * @param P The P tuning constant. Set as 0 to disable.
     * @param I The I tuning constant. Set as 0 to disable.
     * @param D The D tuning constant. Set as 0 to disable.
     * @param T The absolute tolerance of the system.
     * @param C Determines if the loop is continuous.
     */
    public PID(double P, double I, double D, double T, boolean C){
        kP = P;
        kI = I;
        kD = D;
        tolerance = T;
        continuous = C;
        prevTime = Timer.getFPGATimestamp();
    }
    
    /**
     * Sets the output range
     * 
     * @param min The minimum range
     * @param max The maximum range
     */
    public void setOutputRange(double min, double max) {
        rangeOut = false;
    	    minOut = min;
    	    maxOut = max;
    }
    
    /**
    * Sets the input range
    * 
    * @param min The minimum range
    * @param max The maximum range
    */
    public void setInputRange(double min, double max) {
        rangeIn = true;
    	    minIn = min;
    	    maxIn = max;
    }
    
    /**
     * Sets the absolute tolerance.
     * 
     * @param percent The specified encoder distance tolerance.
     */
    public void setTolerance(double distance) { tolerance = distance; }
    
    public boolean onTarget() { return Math.abs(prevMeasure - target) < tolerance; }
    
    /**
     * Sets the target.
     * 
     * @param theTarget The specified distance to go.
     */
    public void setPoint(double theTarget) { target = rangeIn ? clamp(minIn, maxIn, theTarget) : theTarget; }
    
    /**
     * Sets if the loop is continuous.
     * 
     * @param cont is the boolean for the state.
     */
    public void setContinuous(boolean cont) { continuous = cont; }
    
    /**
     * Disables the PID
     * @throws Throwable 
     */
    public void disable() throws Throwable { this.finalize(); }
    
    /**
     * Returns the specified setpoint
     * 
     * @return the setpoint
     */
    public double getSetpoint() { return target; }
    
    /**
     * Returns the specified error
     * 
     * @return the Error
     */
    public double getError() { return prevErr; }
    
    /**
     * Run a single PID calculation on the given inputs. This does not loop
     * itself and must be placed in a loop.
     * 
     * @param measure The current value
     * @return The summation of the P, I, and D operations. Generally used
     * as an output.
     */
    public double calculate(double measure){
        double pValue, iValue, dValue;
        prevMeasure = measure;
        err = target - measure;
        if(continuous && (Math.abs(err) > (maxIn - minIn)/2)){
            err = (err>0) ? (err - maxIn + minIn) : (err + maxIn - minIn);
        }

        if(err * kP < maxOut && err * kP > minOut){ totalError+=err; }
        else                                      { totalError=0; }

        //pValue = Math.abs(currentError) < acceptableRange ? 0: p * currentError;
        pValue = kP * err;
        iValue = kI * totalError * dt();
        dValue = kD * (err - prevErr) / dt();

        prevErr = err;
        return clamp(minOut,maxOut, pValue + iValue + dValue);
    }
    
    private double dt() {
        double time = Timer.getFPGATimestamp();
        double delta = time - prevTime;
        prevTime = time;
        return delta/1000.0;
    }
    
    /**
     * Limits the output to a specified range
     * 
     * @param min The minimum range
     * @param max The maximum range
     * @param value The output to be limited
     * @return The limited values
     */
    public double clamp(double min, double max, double value) {
        return Math.max(min, Math.min(max, value));
    }
    
    public void reset() { target = 0.0; }
}