package frc.robot.subsystems;

import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.PIDValues;

public class chassis extends SubsystemBase{
    public SparkMax LeftMotor, RightMotor;
    public SparkMaxConfig LeftConfig, RightConfig;
    public RelativeEncoder LeftEncoder, RightEncoder;
    public SparkClosedLoopController LeftPID, RightPID;
    public PIDController AprilTagRotationPID, AprilTagDrivePID;
    public PowerDistribution pdh;

    public Field2d field;

    public int SpeedMode = 4; // !FORTEST
    public int MotorInverted = 1;
    public boolean TempAlert, isAprilTagTuning = false;
    
    public JetsonConnection jetson;

    public RobotConfig config;
    public ChassisSpeeds chassisSpeeds;
    public DistanceMeasure ultrasonic = new DistanceMeasure(2, 3);

    public chassis(){
        LeftMotor = new SparkMax(MotorConstants.LeftMotorID, MotorConstants.kMotorType);
        RightMotor = new SparkMax(MotorConstants.RightMotorID, MotorConstants.kMotorType);
        
        LeftEncoder = LeftMotor.getEncoder();
        RightEncoder = RightMotor.getEncoder();
        LeftConfig = new SparkMaxConfig();
        RightConfig = new SparkMaxConfig();

        LeftConfig
            .idleMode(IdleMode.kBrake)
            .inverted(false);
        LeftConfig.closedLoop
            .outputRange(-1, 1)
            .pidf(PIDValues.LeftPID[0],PIDValues.LeftPID[1], PIDValues.LeftPID[2], PIDValues.LeftPID[3]);

        RightConfig
            .idleMode(IdleMode.kBrake)
            .inverted(true);
        RightConfig.closedLoop
            .outputRange(-MotorConstants.kRightMotorVelicotyOffset,MotorConstants.kRightMotorVelicotyOffset)
            .pidf(PIDValues.RightPID[0], PIDValues.RightPID[1], PIDValues.RightPID[2], PIDValues.RightPID[3]);


        LeftMotor.configure(LeftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        RightMotor.configure(RightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        
        LeftPID = LeftMotor.getClosedLoopController();
        RightPID = RightMotor.getClosedLoopController();
        AprilTagRotationPID = new PIDController(PIDValues.AprilTagRotationPID[0], PIDValues.AprilTagRotationPID[1], PIDValues.AprilTagRotationPID[2]);
        AprilTagRotationPID.enableContinuousInput(-180, 180);
        AprilTagDrivePID = new PIDController(PIDValues.AprilTagDrivePID[0], PIDValues.AprilTagDrivePID[1], PIDValues.AprilTagDrivePID[2]);
        
        field = new Field2d();
      
        pdh = new PowerDistribution(0,ModuleType.kCTRE);
        
        jetson = new JetsonConnection("JetsonData");
        chassisSpeeds = new ChassisSpeeds();


        try{
            config = RobotConfig.fromGUISettings();
        }catch(Exception e){
            e.printStackTrace();
        }

    }

    public void drive(double speed, double rotation){
        LeftPID.setReference((speed+rotation)*MotorConstants.kMaxMotorSpeed, ControlType.kVelocity);
        RightPID.setReference((speed-rotation)*MotorConstants.kMaxMotorSpeed, ControlType.kVelocity);
    }

    public void stopMotor(){
        LeftPID.setReference(0, ControlType.kVelocity);
        RightPID.setReference(0, ControlType.kVelocity);
    }

    public Command apriltagTune(){
        return run(() -> {
            double DeltaX = jetson.getAprilTagTX();
            Double rotationSpeed = AprilTagRotationPID.calculate(DeltaX,0);
            drive(0, rotationSpeed > 0.05 ? rotationSpeed : 0.05);
        }).until(() -> jetson.isTagCentered() && jetson.isAprilTag());
    }

    public Command driveToTag(){
        return runOnce(() -> {});
    }

    public Command toggleInverted(){
        return runOnce(() -> MotorInverted = -MotorInverted);
    }

    public Command toggleSpeedMode(boolean inverted){
        return runOnce(() -> SpeedMode += !inverted ? 1 : -1);
    }

    public Pose2d getPose(){
        return LimelightHelpers.getBotPose2d("limelight");
    }

    @Override
    public void periodic(){
        TempAlert = ((LeftMotor.getMotorTemperature()+RightMotor.getMotorTemperature())/2) > MotorConstants.kTempThreshold;

        SmartDashboard.putNumber("RobotSpeed", Math.abs((LeftEncoder.getVelocity()+RightEncoder.getVelocity())/2));
        SmartDashboard.putNumber("LeftMotorSpeed", LeftEncoder.getVelocity());
        SmartDashboard.putNumber("RightMotorSpeed", RightEncoder.getVelocity());
        SmartDashboard.putNumber("MotorTemp", (LeftMotor.getMotorTemperature()+RightMotor.getMotorTemperature())/2);
        SmartDashboard.putBoolean("TempAlert", Math.max(LeftMotor.getMotorTemperature(), RightMotor.getMotorTemperature()) > MotorConstants.kTempThreshold);
        SmartDashboard.putBoolean("MotorInverted", MotorInverted == -1);
        
        SmartDashboard.putBoolean("isEnabled", DriverStation.isEnabled());
        SmartDashboard.putNumber("SpeedMode", SpeedMode);
       
        SmartDashboard.putBoolean("isAprilTagTuning", isAprilTagTuning);

        SmartDashboard.putData(field);
        SmartDashboard.putData(pdh);
        SmartDashboard.putNumber("Distance", ultrasonic.getDistance());
        SmartDashboard.putBoolean("IsUltrasonicEnabled", ultrasonic.isEnabled());
        SmartDashboard.putBoolean("IsDistanceValid",ultrasonic.isRangeValid());
        SmartDashboard.putNumber("WheelSpeedDelta", LeftEncoder.getVelocity()-RightEncoder.getVelocity());
    }
}
