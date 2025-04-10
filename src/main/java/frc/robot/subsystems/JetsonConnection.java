package frc.robot.subsystems;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.JetsonNanoConstants;

public class JetsonConnection extends SubsystemBase {
    public NetworkTable nt;
    public BooleanSubscriber isAprilTag;
    public DoubleSubscriber apriltagTX, apriltagDistance;

    public JetsonConnection(String JetsonNanoTableName){
        nt = NetworkTableInstance.getDefault().getTable(JetsonNanoTableName);
        
        isAprilTag = nt.getBooleanTopic("isAprilTag").subscribe(false, PubSubOption.keepDuplicates(false));
        apriltagTX = nt.getDoubleTopic("aprilTagTX").subscribe(0,PubSubOption.keepDuplicates(false));
        apriltagDistance = nt.getDoubleTopic("apriltagDistance").subscribe(0,PubSubOption.keepDuplicates(false));
    }

    public boolean isAprilTag(){
        return isAprilTag.get();
    }

    public double getAprilTagTX(){
        return apriltagTX.get();
    }

    public boolean isTagCentered(){
        return Math.abs(getAprilTagTX()) < JetsonNanoConstants.kTXTolerance;
    }

    public double getApriltagDistance(){
        return apriltagDistance.get();
    }
}
