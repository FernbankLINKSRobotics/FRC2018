package org.usfirst.frc.team4468.robot.Commands.Routines;

import org.usfirst.frc.team4468.robot.Commands.Drive.StraightDistance;
import org.usfirst.frc.team4468.robot.Commands.Drive.TurnAngle;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.ExpelCube;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.IntakeClamp;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.CommandGroup;


public class CenterAuto extends CommandGroup {

	
    public CenterAuto() {
        addSequential(new IntakeClamp(Value.kForward));
        addSequential(new StraightDistance(-2.0, .5)); //initial movement forward
        if(DriverStation.getInstance().getGameSpecificMessage().charAt(0)=='L') {
            addSequential(new TurnAngle(-80, 10)); //Assuming turning left is negative, -90 degrees
            addSequential(new StraightDistance(-4.0, .5)); //-1.0 is distance from center to middle of our colored side of switch
            addSequential(new TurnAngle(0 ,15)); //Assuming turning right is positive, +90 degrees
            addSequential(new StraightDistance(-4.8, .5)); 
            //Assuming distance needed to travel left is 3 (distance from alliance wall to switch is 4.27m according to game manual)
        } else if (DriverStation.getInstance().getGameSpecificMessage().charAt(0) == 'R') {
            addSequential(new TurnAngle(80, 10)); //Assuming turning left is negative, -90 degrees
             addSequential(new StraightDistance(-3.5, .5)); //-1.0 is distance from center to middle of our colored side of switch
             addSequential(new TurnAngle(0, 15)); //Assuming turning right is positive, +90 degrees
             addSequential(new StraightDistance(-4.3, .5)); 
             //Assuming distance needed to travel left is 3 (distance from alliance wall to switch is 4.27m according to game manual)
        }
        addSequential(new ExpelCube(Value.kReverse, -1.0));
    }
}

// Right auto is a positive mult and the op for left
