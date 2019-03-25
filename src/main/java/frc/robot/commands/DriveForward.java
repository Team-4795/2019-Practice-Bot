/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.triggers.DrivetrainOverride;

public class DriveForward extends InstantCommand {

  private double feet;

  public DriveForward(double feet) {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.drivebase);
    this.feet = feet;
    setTimeout(5.0);
  }

  @Override
  protected void initialize() {
    Robot.drivebase.driveFeet(feet);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.oi.drivetrainOverride.get() || isTimedOut() || Math.abs(Robot.drivebase.getLeftEnc() - (feet * Robot.drivebase.ENCODER_COUNTS_PER_FT)) < Robot.drivebase.allowableError;
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
