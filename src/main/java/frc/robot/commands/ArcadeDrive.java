/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class ArcadeDrive extends Command {
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
    double throttle = 0.9 - (0.55 * Robot.oi.getMainRightTrigger());
    double turn = Robot.oi.getMainLeftJoyY() == 0.0 ? Robot.oi.getMainRightJoyX() * .8 : Robot.oi.getMainRightJoyX() * 0.5;
    SmartDashboard.putNumber("LeftJoy Y", Robot.oi.getMainLeftJoyY());
    SmartDashboard.putNumber("RightJoy X", Robot.oi.getMainRightJoyX());
    Robot.drivebase.setMotors((Robot.oi.getMainLeftJoyY() - turn) * throttle, (Robot.oi.getMainLeftJoyY() + turn) * throttle);

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
