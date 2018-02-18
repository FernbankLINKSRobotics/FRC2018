package org.usfirst.frc.team4468.robot.Commands.Routines;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class PrintStack extends Command {
	private String string;

    public PrintStack(String s) {
       
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println(string);
    }

    // Called repeatedly when this Command is scheduled to run
   

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }
}
