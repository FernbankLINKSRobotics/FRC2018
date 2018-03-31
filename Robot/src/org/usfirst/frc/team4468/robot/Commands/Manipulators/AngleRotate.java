package org.usfirst.frc.team4468.robot.Commands.Manipulators;

import org.usfirst.frc.team4468.robot.Constants;
import org.usfirst.frc.team4468.robot.Robot;
import org.usfirst.frc.team4468.robot.Subsystems.RotatingLift;
import org.usfirst.frc.team4468.robot.Util.PID;
import org.usfirst.frc.team4468.robot.Util.Paths.MotionProfiler;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;

public class AngleRotate extends Command {
	double theta;
	boolean negative;
    private RotatingLift rl = Robot.rotatingLift;
    private PID pid;
    double inc;
    
    public AngleRotate(double angle, double tol) {
        theta = angle;
        /*
        if (angle<0.0) {
        		negative = true;
        }
        else {
        		negative = false;
        }
        System.out.println("PID RotateAngle");
        */

        pid = new PID(Constants.lifterP, Constants.lifterI, Constants.lifterD);
        pid.reset();
        pid.setOutputRange(-1.0, 1.0);
        pid.setAbsTolerance(tol);

        //System.out.println(pid.getSetpoint());
        pid.setPoint(theta);
        
        //// Comment back in these in for added functionality
        //inc = increment;
    }
    
    //private MotionProfiler onedMotion = new MotionProfiler(3.0, 5.0, 3.0, 0.1, theta, negative);
    
    /*
     * Called repeatedly when this Command is scheduled to run (non-Javadoc)
     * 
     * @see edu.wpi.first.wpilibj.command.Command#execute()
     */
    
    protected void initialize() {
    	if(rl.getState() != Value.kForward) {
        	rl.clamp(Value.kForward);
        }
    }
    
    protected void execute() {
    	
        //System.out.println("Executed");
        /*double value;
        if (onedMotion.getDistance(rl.getAngle(), inc)[0]>theta) {
        	value = rl.getAngle();
        }
        else {
        	value = onedMotion.getDistance(rl.getAngle(), inc)[0]-rl.getAngle();
        	System.out.println("value: " + value);
        }*/
        rl.rotate(pid.calculate(rl.getAngle()));
        //System.out.println("Calculated Distance:" + pid.calculate(rl.getAngle()));
    }

    /*
     * Make this return true when this Command no longer needs to run execute()
     * (non-Javadoc)
     * 
     * @see edu.wpi.first.wpilibj.command.Command#isFinished()
     * 
     * @return the command stops when true
     */
    protected boolean isFinished() {
        return pid.onTarget(rl.getAngle()) ||
        	(Robot.oi.ctrl.getRawButton(5) ||
       		 Robot.oi.ctrl.getRawButton(2) ||
       		 Robot.oi.ctrl.getRawButton(3) ||
       		 Robot.oi.ctrl.getRawButton(4));
    }

    /*
     * Called once after isFinished returns true (non-Javadoc)
     * 
     * @see edu.wpi.first.wpilibj.command.Command#end()
     */
    protected void end() {
        rl.clamp(Value.kReverse);
        rl.stop();
        pid.disable();
    }

    /*
     * Called when another command which requires one or more of the same subsystems
     * is scheduled to run (non-Javadoc)
     * 
     * @see edu.wpi.first.wpilibj.command.Command#interrupted()
     */
    protected void interrupted() {
    	//rl.clamp(Value.kReverse);
        rl.stop();
        pid.disable();  
    }
}
