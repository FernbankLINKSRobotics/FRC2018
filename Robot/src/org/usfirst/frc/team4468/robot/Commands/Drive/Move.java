package org.usfirst.frc.team4468.robot.Commands.Drive;

import org.usfirst.frc.team4468.robot.Constants;
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
    private MotionProfiler prof;
    
    private double[] X = {};
    private double[] Y = {};
    private double rightDis = 0;
    private double leftDis = 0;
    
    public Move(double n, Waypoint w, Waypoint ... ws) {
        path = new PathGeneration(w, ws);
        calculatePairs(n, path.getX().size());
        prof = new MotionProfiler(X, Y, 2, 1, 1); // I REALLY HAVE NO IDEA ABOUT THE ACCEL_DISTANCE
        
        for(int i=0; i < n-1; i++) {
            double x = i/path.getX().size();
            distances(prof.execute2D(2, X[i], Y[i])[4], path.omega(x));
            addParallel(new LeftDistance(leftDis));
            addSequential(new RightDistance(rightDis));
        }
    }
    
    private void distances(double v, double o) {
        rightDis += ((2*o + Constants.distanceBetweenWheels*o)*Constants.dt())/(2*Constants.distancePerPulse);
        leftDis  += ((2*o - Constants.distanceBetweenWheels*o)*Constants.dt())/(2*Constants.distancePerPulse);
    }
    
    private void calculatePairs(double n, double l) {
        for(int i=0; i < n-1; i++) {
            Pair<Double, Double> temp = path.apply(i/l);
            X[i] = temp.fst();
            Y[i] = temp.snd();
        }
    }
}