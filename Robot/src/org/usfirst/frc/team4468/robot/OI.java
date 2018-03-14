package org.usfirst.frc.team4468.robot;

import org.usfirst.frc.team4468.robot.Commands.Drive.Shift;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.AngleRotate;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.ExpelCube;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.HoldingRotate;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.IntakeClamp;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.IntakeSpeed;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.Trigger;
import edu.wpi.first.wpilibj.command.Scheduler;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    // Joystick allocation
    public XboxController drvr = new XboxController(Constants.driveController);
    public XboxController ctrl = new XboxController(Constants.operatorController);

    /*
     * Sets up the triggers for different commands and other actions for when a new
     * button is pressed and it ties together the commands and subsystems
     */
    public OI() {
        
    		JoystickButton Trigger = new JoystickButton(ctrl, 1); 
    		JoystickButton LeftButton = new JoystickButton(ctrl, 4); 
    		JoystickButton CenterButton = new JoystickButton(ctrl, 3);
    		JoystickButton BottomButton = new JoystickButton(ctrl, 2);
    		JoystickButton RightButton = new JoystickButton(ctrl, 5);
    		JoystickButton LeftTop = new JoystickButton(ctrl, 6);
    		JoystickButton BottomLeft = new JoystickButton(ctrl, 8);
    		JoystickButton LeftMiddle = new JoystickButton(ctrl, 7);
    		
    		BottomButton.whenPressed(new HoldingRotate(-10.0));
    		CenterButton.whenPressed(new HoldingRotate(-140.0));
    		LeftButton.whenPressed(new AngleRotate(-100.0, -.01));
    		RightButton.whenPressed(new AngleRotate(-60.0, -.01));	
    		LeftMiddle.whenPressed(new IntakeClamp(Value.kReverse));
    		LeftTop.whenPressed(new IntakeClamp(Value.kForward));
    		BottomLeft.whenPressed(new IntakeSpeed(0.7));
    		Trigger.whenPressed(new IntakeSpeed(-.5));
    	    
    		/*
        if (drvr.getTriggerAxis(Hand.kRight) == 1) {
            new Shift(Value.kForward).start();
        } else if (drvr.getTriggerAxis(Hand.kLeft) == 1) {
            new Shift(Value.kReverse).start();
        }
        
        if (ctrl.getAButtonPressed()) {
            new HoldingRotate(0).start();
        } else if (ctrl.getBButtonPressed()) {
            new HoldingRotate(60).start();
        } else if (ctrl.getXButtonPressed()) {
            new HoldingRotate(120).start();
        } else if (ctrl.getYButtonPressed()) {
            new HoldingRotate(140).start();
        }
            
        if (drvr.getBumperPressed(Hand.kRight)) {
            new IntakeClamp(Value.kForward).start();
        } else if (drvr.getBumperPressed(Hand.kLeft)) {
            new IntakeClamp(Value.kReverse).start();
        }
        */
        if (ctrl.getTriggerAxis(Hand.kRight) == 1) {
            Scheduler.getInstance().add(new IntakeSpeed(0.7));
        } 
        //else if (ctrl.getTriggerAxis(Hand.kLeft) == -1 ) {
        //    new IntakeSpeed(0.7).start();
        //}
    }
}
