package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Ultrasonic;

public class DistanceMeasure {
    public Ultrasonic sonic;
    public DistanceMeasure(int trigPin, int echoPin){
        sonic = new Ultrasonic(trigPin, echoPin);
        sonic.setEnabled(true);
    }

    public double getDistance(){
        return sonic.getRangeMM() > 0 ? sonic.getRangeMM()/10/100 : 0;
    }

    public boolean isRangeValid(){
        return sonic.isRangeValid();
    }
    
    public boolean isEnabled(){
        return sonic.isRangeValid();
    }
}
