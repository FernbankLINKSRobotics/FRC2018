package org.usfirst.frc.team4468.robot.Util.Paths;

import org.usfirst.frc.team4468.robot.Util.Matrix;
import java.util.List;

public class PathGeneration {
    
    private Waypoint[] W;
    private List<Quintic> X;
    private List<Quintic> Y;
    
    /* Variatic constructor that takes anyone more than 1 different inputs and stores them to later be generated
     * @param point  this is the necessary first point
     * @param points this is an array of different inputs to the function
     */
    public PathGeneration(Waypoint point, Waypoint... points ) {
        W = new Waypoint[points.length + 1];
        W[0] = point;
        for(int i=1; i < points.length - 1; i++) {
            W[i] = points[i];
        }
    }

    /* This generates the coefficients for 2(N-1) equations that is generated by N waypoints. There are 2 times the number 
     * because the equations because each segment of the path has to be described by 2 equations: one for the X components
     * and another for the Y components. That forms what is called a parametric equation where the X and Y are separated
     * and both represented by a common variable typically t.
     */
    public void generate() {
        /* The first three rows are the general form of a quintic equation except that the real coefficient are 
         * in another vector X where AX=B and the x in the general form is substituted with 0 because each function
         * is in the range 0<=t<=1. The second three when you replace x with 1 showing the end of the function.
         */
        double[][] A = {
                {0, 0, 0, 0, 0, 1},
                {0, 0, 0, 0, 1, 0},
                {0, 0, 0, 2, 0, 0},
                {1 , 1 , 1, 1, 1, 1},
                {5 , 4 , 3, 2, 1, 0},
                {50, 12, 6, 2, 0, 0}
        };
        
        A = Matrix.inverse(A);
        
        for(int i=0; i < W.length - 1; i++) {
            /* append   gens   This Solves a systems of equations between all of the points creating two parameterized equations
             * to the   a new  in the form AX=B where X = inv(A) * B where A is the value of X including shifts from the derivatives
             * lists    poly   and the B is the outputs of all of the equations those being the Vel, Acc and Pos of the beginning 
             * of       from   and end of the functions.
             * polys    out                                                |
             * |        |    ________________________________________________________________________________________________
             * \/       \/   |                                                                                               |
             */
            Y.add(new Quintic(Matrix.multiply(A, new double[] { W[i].y, W[i].Vy, W[i].Ay, W[i+1].y, W[i+1].Vy, W[i+1].Ay })));
            X.add(new Quintic(Matrix.multiply(A, new double[] { W[i].x, W[i].Vx, W[i].Ax, W[i+1].y, W[i+1].Vx, W[i+1].Ax })));
        }
        
    }
    
}
