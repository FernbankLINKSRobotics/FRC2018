package org.usfirst.frc.team4468.robot.Commands.Drive;

import org.usfirst.frc.team4468.robot.Constants;
import org.usfirst.frc.team4468.robot.Util.PID;
import org.usfirst.frc.team4468.robot.Util.Pair;
import org.usfirst.frc.team4468.robot.Util.Paths.MotionProfiler;
import org.usfirst.frc.team4468.robot.Util.Paths.PathGeneration;
import org.usfirst.frc.team4468.robot.Util.Paths.Waypoint;

import edu.wpi.first.wpilibj.command.CommandGroup;



/**
 *
 */
public class Move extends CommandGroup {
    private PathGeneration path;
    private MotionProfiler maintwo;
    private LeftDistance left;
    private RightDistance right;
    
    private double[] X = {
    		/*0,
			0.0254,
			0.0762,
			0.4572,
			0.80645,
			1.143,
			1.362075,
			1.520825,
			1.597025*/
    };
    private double[] Y = {
    		/*0,
			0.4064,
			0.688975,
			1.057275,
			1.400175,
			1.755775,
			2.066925,
			2.5019,
			3.01625*/
    };
    private double rightDis = 0.0;
    private double leftDis = 0.0;
    
    public Move(double n, Waypoint w, Waypoint ... ws) {
    //public Move(double tolerance) {
       path = new PathGeneration(w, ws);
       calculatePairs(n, path.getX().size());
       maintwo = new MotionProfiler(X, Y, 5.0, 6.0, 0.0, 5.0);
       /* for(int i=0; i < n-1; i++) {
            double x = i/path.getX().size();
            distances(prof.execute2D(2, X[i], Y[i])[4], path.omega(x));
            addParallel(new LeftDistance(leftDis));
            addSequential(new RightDistance(rightDis));
        }*/
    	
    	
    }
    public void getDistance(double second) {
    	double distance_left = maintwo.execute2D(second,0,0)[0];
    	double distance_right = maintwo.execute2D(second, 0, 0)[4];
    	left = new LeftDistance(distance_left);
    	right = new RightDistance(distance_right);
    	
    }
    /*private void distances(double v, double o) {
        rightDis += ((2*o + Constants.distanceBetweenWheels*o)*Constants.dt())/(2*Constants.distancePerPulse);
        leftDis  += ((2*o - Constants.distanceBetweenWheels*o)*Constants.dt())/(2*Constants.distancePerPulse);
    }
    */
    private void calculatePairs(double n, double l) {
        for(int i=0; i < n-1; i++) {
            Pair<Double, Double> temp = path.apply(i/l);
            X[i] = temp.fst();
            Y[i] = temp.snd();
        }
    }
}