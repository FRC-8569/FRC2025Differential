package frc.robot.commands;


import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.chassis;

public class ChassisCmd extends Command{
    public chassis chassis; //call the chassis we just written
    public Supplier<Double> DriveSpeedFunc, RotationSpeedFunc,selfAroundFunc; //use the function can called easily

    public ChassisCmd(chassis chassis, Supplier<Double> drive, Supplier<Double> rotation, Supplier<Double> selfAround){
        //initallize the objectes
        this.chassis = chassis;
        this.DriveSpeedFunc = drive;
        this.RotationSpeedFunc = rotation;
        this.selfAroundFunc = selfAround;
        
        //this line is important, you'll addRequirements about the main item to want to controll, here's chassis, in other cases, it might be armController etc.
        addRequirements(chassis);
    }

    //just override it for preventing it initalize something we don't know which may cause some error
    @Override
    public void initialize(){}

    //this function will execute when it is called
    @Override
    public void execute(){
        chassis.drive(
            Math.abs(DriveSpeedFunc.get()) > OIConstants.kControllerDeadBand && !chassis.TempAlert? (double)chassis.SpeedMode/100*chassis.MotorInverted*DriveSpeedFunc.get() : 0, //DriveSpeed with controller tolerance and temperature protection, we set the MotorInveted at here because it will invert the rotation when you setup at drive function
            Math.abs(RotationSpeedFunc.get()) > OIConstants.kControllerDeadBand && !chassis.TempAlert ? 0.04*RotationSpeedFunc.get() : 0 //RotationSpeed with controller tolerance and temperature protection
            );
    }
}
