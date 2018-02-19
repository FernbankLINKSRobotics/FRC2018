package org.usfirst.frc.team4468.robot;

import org.usfirst.frc.team4468.robot.Commands.Drive.Shift;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.IntakeClamp;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.IntakeSpeed;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.Rotate;
import org.usfirst.frc.team4468.robot.Commands.Manipulators.RotateAngle;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    // Joystick allocation
    public XboxController drvr = new XboxController(Constants.driveController);
    public XboxController ctrl = new XboxController(Constants.operatorController);
    
    /* Sets up the triggers for different commands and other actions for when
     * a new button is pressed and it ties together the commands and subsystems
     */
    public OI() {
    		JoystickButton A = new JoystickButton(ctrl, 1); 
    		JoystickButton Y = new JoystickButton(ctrl, 4); 
    		JoystickButton X = new JoystickButton(ctrl, 3);
    		JoystickButton B = new JoystickButton(ctrl, 2);
    		JoystickButton LB = new JoystickButton(ctrl, 5);
    		JoystickButton RB = new JoystickButton(ctrl, 6);
    		JoystickButton ST = new JoystickButton(ctrl, 8);
    		JoystickButton BK = new JoystickButton(ctrl, 7);
    		
    		A.whenPressed(new RotateAngle(0.0));
    		Y.whenPressed(new RotateAngle(-140.0));
    		X.whenPressed(new RotateAngle(-120.0));
    		B.whenPressed(new RotateAngle(-60.0));	
    		LB.whenPressed(new IntakeClamp(Value.kReverse));
    		RB.whenPressed(new IntakeClamp(Value.kForward));
    		ST.whenPressed(new IntakeSpeed(0.7));
    		BK.whenPressed(new IntakeSpeed(-.7));
    	
    		
        /*if (drvr.getTriggerAxis(Hand.kRight) == 1) {
            new Shift(Value.kForward);
        } else if (drvr.getTriggerAxis(Hand.kLeft) == 1) {
            new Shift(Value.kReverse);
        }
        
        if (ctrl.getAButtonPressed()) {
            new RotateAngle(0);
        } else if (ctrl.getBButtonPressed()) {
            new RotateAngle(60);
        } else if (ctrl.getXButtonPressed()) {
            new RotateAngle(120);
        } else if (ctrl.getYButtonPressed()) {
            new RotateAngle(180);
        }
            
        if (drvr.getBumperPressed(Hand.kRight)) {
            new IntakeClamp(Value.kForward);
        } else if (drvr.getBumperPressed(Hand.kLeft)) {
            new IntakeClamp(Value.kReverse);
        }
        
        if (ctrl.getTriggerAxis(Hand.kRight) > 5) {
            new IntakeSpeed(1);
        } else if (ctrl.getTriggerAxis(Hand.kLeft) < -5 ) {
            new IntakeSpeed(-1);
        }
        
        if(drvr.getAButton()) {
            new Rotate(false);
        } else if(drvr.getYButton()){
            new Rotate(true);
        }
        */
    }
}
