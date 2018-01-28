package org.usfirst.frc.team4468.robot.Util;

public class Pair<X, Y> {
    private X x;
    private Y y;
    
    public Pair(X i, Y j) {
        this.x = i;
        this.y = j;
    }
    
    public X fst() {
        return x;
    }
    
    public Y snd() {
        return y;
    }
    
    public void setFst(X k) {
        this.x = k;
    }
    
    public void setSnd(Y l) {
        this.y = l;
    }
}