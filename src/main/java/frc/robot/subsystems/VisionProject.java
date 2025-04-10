package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConstants;

public class VisionProject{
    public enum VisionType{
        kJetsonNano,
        kLimeLight;
    }

    public VisionType visionType;
    public String name;
    public String Port;
    public String TableName = "";
    public HttpCamera videoSource;
    public BooleanSubscriber isAprilTagSub;
    public DoubleSubscriber aprilTagTX, aprilTagDistance;

    public VisionProject(String DeviceName, String Port, VisionType type){
        this.visionType = type;
        this.name = DeviceName;
        this.Port = Port;
        switch (visionType) {
            case kJetsonNano:
                JetsonInit();
                break;
            case kLimeLight:
                LimelightInit();
                break;
        }
    }

    private void LimelightInit(){
        videoSource = new HttpCamera(name, String.format("http://{}.local:{}", name, Port));
        videoSource.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
        CameraServer.addCamera(videoSource);
    }

    public void JetsonInit(){
        videoSource = new HttpCamera(name, String.format("http://{}.local:{}", name,Port));
        videoSource.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
        CameraServer.addCamera(videoSource);
        NetworkTable nt = NetworkTableInstance.getDefault().getTable(TableName);
        isAprilTagSub = nt.getBooleanTopic("isAprilTag").subscribe(false, PubSubOption.keepDuplicates(false));
        aprilTagTX = nt.getDoubleTopic("aprilTagTX").subscribe(0, PubSubOption.keepDuplicates(false));
        aprilTagDistance = nt.getDoubleTopic("aprilTagDistance").subscribe(0, PubSubOption.keepDuplicates(false));
    }

    public void setTableName(String tableName){
        this.TableName = tableName;
    }

    public boolean isAprilTag(){
        switch (visionType) {
            case kJetsonNano:
                return isAprilTagSub.get();

            case kLimeLight:
                return LimelightHelpers.getTV(name);

            default:
                return false;
        }
    }

    public double rotationToTag(){
        switch (visionType) {
            case kJetsonNano:
                return aprilTagTX.get();
            
            case kLimeLight:
                return LimelightHelpers.getTX(name);
            default:
                return 0;
        }
    }

    public double distanceToTag(){
        switch (visionType) {
            case kJetsonNano:
                return aprilTagDistance.get();
                
            case kLimeLight:
                return LimelightHelpers.getBotPose(name).length == 6 ? LimelightHelpers.getBotPose(name)[0] : 0;
            default:
                return 0;
        }
    }

    public boolean isTagCentered(){
        switch (visionType) {
            case kJetsonNano:
                return rotationToTag() < VisionConstants.kJetsonRotationTolerance;
            
            case kLimeLight:
                return rotationToTag() < VisionConstants.kLimelightRotationTolerance;
        
            default:
                return true;
        }
    }

    public boolean isTagArrived(){
        switch (visionType) {
            case kJetsonNano:
                return distanceToTag() < VisionConstants.kJetsonDriveTolerance;
            
            case kLimeLight:
                return rotationToTag() < VisionConstants.kLimelightDriveTolerance;
        
            default:
                return true;
        }
    }

    public Pose2d getRobotPose(){
        return visionType == VisionType.kLimeLight ? LimelightHelpers.getBotPose2d(name) : null;
    }
}