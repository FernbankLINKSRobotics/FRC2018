package org.usfirst.frc.team4468.robot.Commands.Routines;

import org.usfirst.frc.team4468.robot.Robot;
import org.usfirst.frc.team4468.robot.Commands.Drive.LeftDistance;
import org.usfirst.frc.team4468.robot.Commands.Drive.RightDistance;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class Run extends CommandGroup {
	//public static LeftDistance leftDistance = new LeftDistance(2);
	//public static RightDistance rightDistance = new RightDistance(2);

    public Run() {
    }
    
    public void llama() {
    		System.out.println("Llama started"); /*
		//addSequential(new PrintStack("test"));
 */
    		//addSequ(new LeftDistance(2));
    		//addSequential(new RightDistance(2));
    		Robot.drive.drive(0, .3);
    }
}
