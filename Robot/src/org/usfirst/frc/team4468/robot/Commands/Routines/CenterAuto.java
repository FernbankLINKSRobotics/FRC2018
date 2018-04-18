package org.usfirst.frc.team4468.robot.Commands.Routines;

import org.usfirst.frc.team4468.robot.Commands.Drive.StraightDistance;
import org.usfirst.frc.team4468.robot.Commands.Drive.TurnAngle;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.AngleRotate;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.ExpelCube;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.IntakeClamp;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.CommandGroup;


public class CenterAuto extends CommandGroup {

	
    public CenterAuto() {
    	
    	System.out.println("IN CENTERAUTO");
        addSequential(new IntakeClamp(Value.kForward));
        addSequential(new StraightDistance(-1.25, .5)); //initial movement forward
        addSequential(new TurnAngle(-80, 10)); //Assuming turning left is negative, -90 degrees
        System.out.println("Completed First Turn");
        addSequential(new StraightDistance(-3.9, .5)); //-1.0 is distance from center to middle of our colored side of switch
        System.out.println("Completed second move forward");
        addSequential(new TurnAngle(-10 ,10)); //Assuming turning right is positive, +90 degrees
        addSequential(new StraightDistance(-6.1, .7)); 
        //Assuming distance needed to travel left is 3 (distance from alliance wall to switch is 4.27m according to game manual)
        addSequential(new AngleRotate(-140, 10));
        addSequential(new ExpelCube(Value.kReverse, -0.7));
// Right auto is a positive mult and the op for left

    }
}
