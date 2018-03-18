package org.usfirst.frc.team4468.robot.Commands.Routines;

import org.usfirst.frc.team4468.robot.Commands.Drive.StraightDistance;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.AngleRotate;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.HoldingRotate;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.IntakeClamp;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.IntakeSpeed;
import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class CenterAuto extends CommandGroup {

	
    public CenterAuto() {
        System.out.println("IN RUN");
        //addSequential(new IntakeClamp(Value.kForward));
        addSequential(new StraightDistance(-2.5, 0.5));
        addSequential(new AngleRotate(-140.0, 20));
        //addParallel(new HoldingRotate(-140.0));
        addSequential(new IntakeSpeed(-1.0));
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
