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

public class ManualHatchControl extends Command {

  public ManualHatchControl() {
    requires(Robot.hatch);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.hatch.servoUp = true;
    Robot.hatch.hatchUp = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    boolean hatchActuallyUp = Robot.hatch.hatchMotor.getSensorCollection().isRevLimitSwitchClosed();
    boolean hatchActuallyDown = Robot.hatch.hatchMotor.getSensorCollection().isFwdLimitSwitchClosed();

    if (Robot.oi.getMainBButtonPressed()) {
      Robot.hatch.servoUp = !Robot.hatch.servoUp;
    }
    if (Robot.oi.getMainLeftBumper()) {
      Robot.hatch.hatchUp = true;
    } 

    if (hatchActuallyUp || Robot.hatch.servoUp) {
      Robot.hatch.hatchUp = false;
    }
    if (Robot.hatch.hatchUp) {
      //Robot.hatch.servoUp = false;
      Robot.hatch.setRamp(0.0);
      Robot.hatch.set(-1.0);
    } else if (hatchActuallyDown) {
      Robot.hatch.set(0.0);
    } else {
      Robot.hatch.setRamp(0.6);
      Robot.hatch.set(0.1);
    }

    SmartDashboard.putBoolean("Forward Limit", hatchActuallyUp);
    SmartDashboard.putBoolean("Reverse Limit", hatchActuallyDown);
    SmartDashboard.putBoolean("servoUp", Robot.hatch.servoUp);
    SmartDashboard.putBoolean("hatchUp", Robot.hatch.hatchUp);

    Robot.hatch.setServoUp(Robot.hatch.servoUp);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false; //change to getRevLimitSwitch if needed later
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
