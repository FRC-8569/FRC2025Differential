package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Constants {
    public static class ChassisConstants{
        public static final double kWheelCirc = (6*2.54)*Math.PI/100;
    }

    public static class OIConstants {
        public static final double kControllerDeadBand = 0.03;
    }

    public static class MotorConstants{
        public static final int LeftMotorID = 5;
        public static final int RightMotorID = 8;
        public static final MotorType kMotorType = MotorType.kBrushless;

        public static final double kMaxMotorSpeed = 5676;
        public static final double kGearRatio = 3.0/7;
        public static final double kMaxRobotSpeed = kMaxMotorSpeed*kGearRatio*ChassisConstants.kWheelCirc/60;
        public static final double kMaxAccel = 79.5/17;

        public static final double kMotorKv = 473;
        public static final int kTempThreshold = 60;

        public static final double kRightMotorVelicotyOffset = 1;
    }

    public static class PIDValues {
        public static final double[] LeftPID = {0.00045, 0, 0.0001,1/MotorConstants.kMotorKv};
        public static final double[] RightPID =  {0.00044, 0, 0,1/MotorConstants.kMotorKv};
        public static final double[] AprilTagRotationPID = {0.3,0,0};
        public static final double[] AprilTagDrivePID = {0.05,0,0};
    }

    public static class LimeLightConstants{
        public static final String kLeftLimelightName = "limelight-left";
        public static final String kLeftLimelightConnectionWay = "http://10.85.69.88:5800";
        public static final String kRightLimelightName = "limelight";
        public static final String kRightLimelightConnectionWay = "http://10.85.69.87:5800";

        public static final double kHorizonalTuneTolerance = 1;
        public static final double kRotationSpeedThreshold = 0.1;
        public static final double kDriveSafeDistance = 0.2;
    }

    public static class otherConstants {
        public static final int[] ultrasonicPins = {2,3};
        
    }

    public static class  JetsonNanoConstants {
        public static final String kNanoIP = "10.85.69.90";
        public static final double kTXTolerance = 10;
        
    }

    

    public static class VisionConstants {
        public static final double kJetsonRotationTolerance = 10;
        public static final double kLimelightRotationTolerance = 1;
        public static final double kJetsonDriveTolerance = 0.2;
        public static final double kLimelightDriveTolerance = 20;
        
    }
}
