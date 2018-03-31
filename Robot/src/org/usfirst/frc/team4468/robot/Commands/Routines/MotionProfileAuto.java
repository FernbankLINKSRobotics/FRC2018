package org.usfirst.frc.team4468.robot.Commands.Routines;

import org.usfirst.frc.team4468.robot.Commands.Drive.Move;
import org.usfirst.frc.team4468.robot.Util.Paths.Waypoint;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class MotionProfileAuto extends CommandGroup {
	
	Waypoint point_one = new Waypoint(0, 0, 1, 0, 1, 0);
	Waypoint point_two = new Waypoint(0.903225, 1.4750875, 1, 1, 1, 1);
	Waypoint point_three = new Waypoint(1.597025, 3.01625, 0, 1, 0, 1);
	
    public MotionProfileAuto() {
    	    addSequential(new Move(9, point_one, point_two, point_three));
    }
}
