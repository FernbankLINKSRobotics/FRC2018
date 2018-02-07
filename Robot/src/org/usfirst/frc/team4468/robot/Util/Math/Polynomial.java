package org.usfirst.frc.team4468.robot.Util.Math;

public class Polynomial {

    private double[] coefficient;
    
    //// Constructors
    /* Variatic constructor that takes anyone more than 1 different inputs and stores them to later be generated
     * @param v  this is the necessary first point
     * @param vs this is an array of different inputs to the function
     */
    public Polynomial(double v, double ... vs) {
        coefficient = new double[vs.length + 1];
        coefficient[0] = v;
        for(int i=0; i < vs.length; i++) {
            coefficient[i+1] = vs[i];
        }
    }
    
    /* Optional constructor for just arrays better for matrix math
     * @param a An aray of coefficients
     */
    public Polynomial(double[] a) {
        coefficient = a;
    }
    
    
    
    //// The public accessible calculations
    /* This function applies a values to the polynomial
     * @param x the values passed into the polynomial
     * @return the y value at a given x
     */
    public double apply(double x) {
        return applyImpl(coefficient, x);
    }
    
    /* Defines the value of the Nth derivative of the polynomial at a point
     * @param n the order of the derivative taken
     * @param x the value that is fed into the new polynomial
     * @return the values of the nth derivative at a given point
     */
    public double nthDerivativeVal(double n, double x) {
        return applyImpl(nthDerivative(n), x);
    }
    
    /* Defines the Nth derivative of the polynomial
     * @param n the order of the derivative taken
     * @return An array of doubles for the coefficients of the derivative
     */
    public double[] nthDerivative(double n) {
        double[] ret = coefficient;
        int i = 0;
        
        do {
            if(ret.length > 0) {
                ret = derivativeImpl(ret);
                i++;
            } else {
                break;
            }
        } while(i < n);
        
        return ret;
    }
   
    
    
    //// Implementation
    /* The general for the derivative given a list of coefficients
     * @param c a list of coefficients for the input polynomial
     * @return the coefficients of the derivative of the input
     */
    private double[] derivativeImpl(double[] c) {
        
        double[] derivative = new double[c.length - 1];
        for(int i=0;i<derivative.length;i++){
            derivative[i] = c[i]*(c.length - 1 -i );
        }
        return derivative;
    }
    
    /* Applies a value to a list that would represent the coefficients of a polynomial
     * @param a an array of coefficients
     * @param x the input to the polynomial
     * @return the y value of the input curve at the inputted point
     */
    private double applyImpl(double[] a, double x) {
        double acc = 0;
        for(int i=0; i < a.length; i++) {
            acc += a[i] * Math.pow(x, i);
        }
        return acc;
    }
}
