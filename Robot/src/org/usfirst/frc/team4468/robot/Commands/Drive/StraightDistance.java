package org.usfirst.frc.team4468.robot.Commands.Drive;

import org.usfirst.frc.team4468.robot.Constants;
import org.usfirst.frc.team4468.robot.Robot;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.AngleRotate;
import org.usfirst.frc.team4468.robot.Subsystems.Drivetrain;
import org.usfirst.frc.team4468.robot.Subsystems.RotatingLift;
import org.usfirst.frc.team4468.robot.Util.PID;
import edu.wpi.first.wpilibj.command.Command;

public class StraightDistance extends Command {
    private double distance;
    private double tolerance;
    
    private Drivetrain dt = Robot.drive;
    private PID pid;
    
    //private RotatingLift rl = Robot.rotatingLift;
    //private AngleRotate angleRotate;
        
    public StraightDistance(double d, double tol) {
        System.out.println("PID Right Started");
        requires(dt);
        distance = d;
        tolerance = tol;
            
        pid = new PID(Constants.lineP, Constants.lineI, Constants.lineD);
        pid.setOutputRange(-1.0, 1.0);
        pid.setAbsTolerance(tolerance);
        pid.setPoint(distance);
        //angleRotate =new AngleRotate(rl.getAngle(), 0.0);
    }
        
    /* Called repeatedly when this Command is scheduled to run
     * (non-Javadoc)
     * @see edu.wpi.first.wpilibj.command.Command#execute()
     */
    protected void execute() {
    		/*
        System.out.println("Inside Line execute");
        System.out.println("Line Output: " + pid.calculate(dt.getLeftDistance()));
        System.out.println("Line Error: " + pid.getError());
        System.out.println("Line SetPoint: " + pid.getSetpoint());
        System.out.println("Distance: " + dt.getDis());
        */
        dt.arcade(0.0, pid.calculate(dt.getDis()));
    }

    /* Make this return true when this Command no longer needs to run execute()
     * (non-Javadoc)
     * @see edu.wpi.first.wpilibj.command.Command#isFinished()
     * @return the command stops when true
     */
    protected boolean isFinished() {
        System.out.println("pid target " + pid.onTarget(dt.getDis()));
        //System.out.println("Is Done");
        return pid.onTarget(dt.getDis());
    }

    /* Called once after isFinished returns true
     * (non-Javadoc)
     * @see edu.wpi.first.wpilibj.command.Command#end()
     */
    protected void end() {
        System.out.println("End");
        dt.stop();
    }

    /* Called when another command which requires one or more of the same
     * subsystems is scheduled to run
     * (non-Javadoc)
     * @see edu.wpi.first.wpilibj.command.Command#interrupted()
     */
    protected void interrupted() {
        dt.stop();
    }
}
