package org.usfirst.frc.team4468.robot.Commands.Manipulators;

import org.usfirst.frc.team4468.robot.Constants;
import org.usfirst.frc.team4468.robot.Robot;
import org.usfirst.frc.team4468.robot.Subsystems.RotatingLift;
import org.usfirst.frc.team4468.robot.Util.PID;
import org.usfirst.frc.team4468.robot.Util.Paths.MotionProfiler;

import edu.wpi.first.wpilibj.command.Command;

public class AngleRotate extends Command {
	double theta;
	boolean negative;
    private RotatingLift rl = Robot.rotatingLift;
    private PID pid;
    
    public AngleRotate(double angle) {
        theta = angle;
        if (angle<0) {
        	negative = true;
        }
        else {
        	negative = false;
        }
        System.out.println("PID RotateAngle");

        pid = new PID(Constants.lifterP, Constants.lifterI, Constants.lifterD);
        pid.reset();
        pid.setOutputRange(-1.0, 1.0);
        pid.setPerTolerance(.5);

        System.out.println(pid.getSetpoint());
        pid.setPoint(theta);
        System.out.println("setpoint one: " + pid.getSetpoint());
    }
    
    private MotionProfiler onedMotion = new MotionProfiler(3.0, 5.0, 3.0, 0.1, theta, negative);
    
    /*
     * Called repeatedly when this Command is scheduled to run (non-Javadoc)
     * 
     * @see edu.wpi.first.wpilibj.command.Command#execute()
     */
    
    protected void execute(double time) {
        System.out.println("Executed");
        rl.rotate(pid.calculate(onedMotion.execute1D(time)[0]));
        System.out.println("Calculated Distance:" + pid.calculate(onedMotion.execute1D(time)[0]));
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
        return pid.onTarget(rl.getAngle());
    }

    /*
     * Called once after isFinished returns true (non-Javadoc)
     * 
     * @see edu.wpi.first.wpilibj.command.Command#end()
     */
    protected void end() {
        rl.stop();
    }

    /*
     * Called when another command which requires one or more of the same subsystems
     * is scheduled to run (non-Javadoc)
     * 
     * @see edu.wpi.first.wpilibj.command.Command#interrupted()
     */
    protected void interrupted() {
        rl.stop();
    }
}
