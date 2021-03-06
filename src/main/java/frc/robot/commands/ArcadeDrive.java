/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Robot;

public class ArcadeDrive extends Command {
  boolean cameraToggle = false;

  public ArcadeDrive() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.drivebase);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double throttle = 0.55 - (0.3 * Robot.oi.getMainRightTrigger());
    double turn = Robot.oi.getMainLeftJoyY() == 0.0 ? Robot.oi.getMainRightJoyX() * .8 : Robot.oi.getMainRightJoyX() * 0.5;
    SmartDashboard.putNumber("LeftJoy Y", Robot.oi.getMainLeftJoyY());
    SmartDashboard.putNumber("RightJoy X", Robot.oi.getMainRightJoyX());
    
    if (Robot.oi.getMainRightTrigger() > .5) {
      turn *= 1.3;
    }

    Robot.drivebase.setMotors((Robot.oi.getMainLeftJoyY() - turn) * throttle, (Robot.oi.getMainLeftJoyY() + turn) * throttle);

    if (Robot.oi.getMainAButtonPressed()) {
      Robot.drivebase.resetEnc();
      //cameraToggle = !cameraToggle;
      //NetworkTableInstance.getDefault().getEntry("CamID").setNumber(cameraToggle ? 0 : 1);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drivebase.setMotors(0.0, 0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
