package org.usfirst.frc.team4468.robot.Subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team4468.robot.Constants;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import org.usfirst.frc.team4468.robot.Commands.Drive.TeleOp;
/**
 *
 */
public class Drivetrain extends Subsystem {
    //// Declarations
    // Class Declarations
    private SpeedControllerGroup leftMotors = new SpeedControllerGroup(
            new VictorSP(Constants.leftTop),
            new VictorSP(Constants.leftMid),
            new VictorSP(Constants.leftBot)
    );
    
    private SpeedControllerGroup rightMotors = new SpeedControllerGroup(
            new VictorSP(Constants.rightTop),
            new VictorSP(Constants.rightMid),
            new VictorSP(Constants.rightBot)
    );
    
    private Encoder leftEncoder = new Encoder(
            Constants.leftEnc1, 
            Constants.leftEnc2, 
            Constants.leftEncInverted, 
            Encoder.EncodingType.k4X
    );
    
    private Encoder rightEncoder = new Encoder(
            Constants.rightEnc1, 
            Constants.rightEnc2, 
            Constants.rightEncInverted,
            Encoder.EncodingType.k4X
    );
    
    private AHRS gyro = new AHRS(Constants.gyroPort);
    
    // Variable Declarations
    private double leftPrevVel = 0;
    private double rightPrevVel = 0;
    private double prevAngularVel = 0;
    
    
    
    //// Class Inits
    // Automatic actions needed for the rest of the computation
    public Drivetrain() {
        leftMotors.setInverted(true);
    }
    
    /* This is saying that when the robot starts up this sub calls
     * the command for general tank drive
     * (non-Javadoc)
     * @see edu.wpi.first.wpilibj.command.Subsystem#initDefaultCommand()
     */
    public void initDefaultCommand() {
        setDefaultCommand(new TeleOp());
    }
    
    
    
    //// Drive Motor Code
    /* Sets the motors in for a tank drive
     * @param left  the values to the left side of the drive train
     * @param right the right values to the right side
     */
    public void drive(double left, double right) {
        leftMotors .set((left + right)/2);
        rightMotors.set((left - right)/2);
    }
    
    /* Stops all of the motors
     */
    public void stop() {
        leftMotors .stopMotor();
        rightMotors.stopMotor();
    }
    
    
    
    //// Encoder Getters
    /* gets the displacement of the robot
     * @return The average distance between the motors
     */
    public double getDis() {
        return ave(leftEncoder.getDistance(), rightEncoder.getDistance());
    }
    
    /* Gets the velocity of the robot
     * @return The average velocity between the motors
     */
    public double getVel() {
        return ave(leftEncoder.getRate(), rightEncoder.getRate());
    }
    
    /* Gets the acceleration of the robot
     * @return The average acceleration between the motors
     */
    public double getAcc() {
        double leftVel  = leftEncoder .getRate();
        double rightVel = rightEncoder.getRate();
        
        leftPrevVel  = leftVel;
        rightPrevVel = rightVel;
        
        return ave((leftVel - leftPrevVel), (rightVel- rightPrevVel));
    }
    
    /* gets the displacement of the right side
     * @return The number of pulses of the right side
     */
    public double getRightDistance() {
        return rightEncoder.getDistance();
    }
    
    /* gets the displacement of the left side
     * @return The number of pulses of the left side
     */
    public double getLeftDistance() {
        return leftEncoder.getDistance();
    }
    
    
    /* Converts pulses to inches
     * @param p the number of encoder pulses
     * @return the number of inches 
     */
    public double pulsesToDistance(double p) {
        return p * Constants.distancePerPulse;
    }
    
    /* Average
     * @return the average of two values
     */
    private double ave(double left, double right) {
        return (left + right) / 2;
    }
    
    
    
    //// Gyro Readings
    /* The gyro angle of the robot
     * @return the angle in degrees
     */
    public double getAngle() {
        return gyro.getAngle();
    }
    
    /* The robot's angular velocity
     * @return degrees/sec 
     */
    public double getAngularVel() {
        return gyro.getRate();
    }
    
    /* The robot's angular acceleration
     * @return degrees/sec^2
     */
    public double getAngularAcc() {
        double angularVel = gyro.getRate();
        prevAngularVel = angularVel;
        return angularVel - prevAngularVel;
    }
    
    
    
    //// Resets Sensors
    // Resets the encoders to Zero
    public void encoderReset() {
        leftEncoder .reset();
        rightEncoder.reset();
    }
    
    // resets the gyro back to Zero
    public void gyroReset() {
        gyro.reset();
    }
}

