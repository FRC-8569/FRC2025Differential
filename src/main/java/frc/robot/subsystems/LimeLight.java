package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimeLightConstants;

public class LimeLight {
    public String LimeLightName;

    public LimeLight(String limelight, String ConnectionWay){
        LimeLightName = limelight;
        HttpCamera camera = new HttpCamera(limelight, ConnectionWay);
        camera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
        CameraServer.addCamera(camera);
        }

    public boolean isAprilTag(){
        return LimelightHelpers.getTV(LimeLightName);
    }

    public double rotationToTag(){
        return LimelightHelpers.getTX(LimeLightName);
    }

    public boolean isTagCentered(){
        return LimeLightConstants.kHorizonalTuneTolerance > Math.abs(rotationToTag());
    }

    public double distanceToTag(){
        return isAprilTag() ? LimelightHelpers.getBotPose(LimeLightName)[0] : 3000;
    }

    public double[] robotPose(){
        return LimelightHelpers.getBotPose(LimeLightName);
    }

    public boolean isTagArrived(){
        return Math.abs(distanceToTag()) < LimeLightConstants.kDriveSafeDistance;
    }

    public Pose2d getPose(){
        return LimelightHelpers.getBotPose2d(LimeLightName);
    }
}