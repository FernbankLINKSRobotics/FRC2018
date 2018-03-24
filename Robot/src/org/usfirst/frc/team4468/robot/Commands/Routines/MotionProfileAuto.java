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
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
    	addSequential(new Move(9, point_one, point_two, point_three));
    	
    }
}
