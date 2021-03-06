package org.usfirst.frc.team4468.robot.Commands.Drive;

import org.usfirst.frc.team4468.robot.Constants;
import org.usfirst.frc.team4468.robot.Robot;
import org.usfirst.frc.team4468.robot.Util.PID;
import org.usfirst.frc.team4468.robot.Subsystems.Drivetrain;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Uses the gyro to turn the robot to a spocifc angle
 */
public class TurnAngle extends Command {
    
    private double theta;
    private double tolerance;
        
    private Drivetrain dt = Robot.drive;
    private PID pid;
        
    public TurnAngle(double t, double tol) {
        requires(dt);
        theta = t;
        tolerance = tol;
        
        //if(t < 0) {
        	pid = new PID(Constants.angleP
        				, Constants.angleI
        				, Constants.angleD
        				, tol
        				, true);
        /*} else {
        	pid = new PID(Constants.angleP
        				, Constants.angleI
        				, Constants.angleD);
        }*/
        pid.setOutputRange(-1.0, 1.0);
        pid.setPoint(theta);
        
        System.out.println("TURN START");
    }
        
    /* Called repeatedly when this Command is scheduled to run
     * (non-Javadoc)
     * @see edu.wpi.first.wpilibj.command.Command#execute()
     */
    protected void execute() {
        //System.out.println("Angle:" + dt.getAngle());
        //System.out.println("Motor Out:" + pid.calculate(dt.getAngle()));
        dt.arcade(pid.calculate(dt.getAngle()), 0.0);
    }

    /* Make this return true when this Command no longer needs to run execute()
     * (non-Javadoc)
     * @see edu.wpi.first.wpilibj.command.Command#isFinished()
     * @return the command stops when true
     */
    protected boolean isFinished() {
        //System.out.println("Isfinished: " + pid.onTarget(dt.getAngle()));
    	//System.out.println("Turning");
        return pid.onTarget();
    }

    /* Called once after isFinished returns true
     * (non-Javadoc)
     * @see edu.wpi.first.wpilibj.command.Command#end()
     */
    protected void end() {
        System.out.println("TURN END");
        dt.stop();
    }

    /* Called when another command which requires one or more of the same
     * subsystems is scheduled to run
     * (non-Javadoc)
     * @see edu.wpi.first.wpilibj.command.Command#interrupted()
     */
    protected void interrupted() {
        System.out.println("TURN INTER");
        dt.stop();
    }
}
