// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ChassisCmd;
import frc.robot.subsystems.chassis;

public class RobotContainer {
  public chassis chassis = new chassis(); //define the chassis object
  public Joystick controller = new Joystick(0); //define the joystick to controll the robot

  public RobotContainer() {
    //set the default fommand for driving the robot
    chassis.setDefaultCommand(new ChassisCmd(
      chassis,
      () -> controller.getRawAxis(1), 
      () -> -controller.getRawAxis(0),
      () -> controller.getRawAxis(3))
      );
    configureBindings();
  }

  private void configureBindings() {
    //side features
    new Trigger(() -> controller.getRawButton(5) && chassis.SpeedMode != 0)
        .onTrue(chassis.toggleSpeedMode(true)); //decrease speed
    new Trigger(() -> controller.getRawButton(6) && chassis.SpeedMode != 10)
        .onTrue(chassis.toggleSpeedMode(false)); //increase speed
    new Trigger(() -> controller.getRawButton(3))
        .onTrue(chassis.toggleInverted());
    new Trigger(() -> controller.getRawButton(1))
        .onTrue(chassis.apriltagTune());
    new Trigger(() -> controller.getRawButton(2))
        .onTrue(chassis.driveToTag());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
