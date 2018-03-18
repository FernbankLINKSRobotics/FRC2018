package org.usfirst.frc.team4468.robot.Commands.Routines;

import org.usfirst.frc.team4468.robot.Commands.Drive.StraightDistance;
import org.usfirst.frc.team4468.robot.Commands.Drive.TurnAngle;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.AngleRotate;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.HoldingRotate;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.IntakeClamp;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.IntakeSpeed;
import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class CenterAuto extends CommandGroup {

	
    public CenterAuto() {
        addSequential(new IntakeClamp(Value.kReverse));
        addParallel(new AngleRotate(-140.0,20));
        addSequential(new StraightDistance(-1.0, .5)); //initial movement forward
        if(DriverStation.getInstance().getGameSpecificMessage().charAt(0) == 'L') {
        		addSequential(new TurnAngle(-90, 5)); //Assuming turning left is negative, -90 degrees
        		addSequential(new StraightDistance(-1.0, .05)); //-1.0 is distance from center to middle of our colored side of switch
        		addSequential(new TurnAngle(90,5)); //Assuming turning right is positive, +90 degrees
        } else if (DriverStation.getInstance().getGameSpecificMessage().charAt(0) == 'R') {
        		addSequential(new TurnAngle(90,5));
        		addSequential(new StraightDistance(-1.0, 5));
        		addSequential(new TurnAngle(90,5));
        }
        addSequential(new StraightDistance(-3.00, .5)); 
        //Assuming distance needed to travel left is 3 (distance from alliance wall to switch is 4.27m according to game manual)
        addSequential(new IntakeSpeed(-.7));
        addSequential(new IntakeClamp(Value.kForward));
    }
}


/* SUDO CODE
 * Clamo
 * Move to angle x
 * Go froward -x distance (starting facing wall)
if(DriverStation.getdata.charat(0) == 'L' {
turn negative -90
go y distance
turn positive 90
go forward z distance
deposit cube (unclamo, rev intake)
} else if(all that stuff prior except for right) {
do everything again only for right side
}


see line 48 of OI.java for another change
 */
