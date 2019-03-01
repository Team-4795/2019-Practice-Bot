/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.ManualHatchControl;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */

public class Hatch extends Subsystem {
  public final TalonSRX hatchMotor;

  public Hatch() {
    hatchMotor = new TalonSRX(RobotMap.HATCH_MOTOR.value);

    Robot.initTalon(hatchMotor);
    this.setRamp(0.0);
    this.set(0.0);
  }

  public void setRamp(double rate) {
    hatchMotor.configOpenloopRamp(rate);
  }

  public void set(double speed) {
    hatchMotor.set(ControlMode.PercentOutput, speed);
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new ManualHatchControl());
  }
}