/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.DriveForward;
import frc.robot.triggers.DrivetrainOverride;

/**
 * Add your docs here.
 */
public class OI {

    private static final double DEADZONE = 0.15 ;
  
    private Joystick MAIN_CONTROLLER, ARM_CONTROLLER;
    private JoystickButton XButton, YButton;
    private double value;
    private JoystickButton ArmBButton, ArmLeftBumper, AButton, BButton;
    public DrivetrainOverride drivetrainOverride;

  
    public OI() { 
      
    }

    public void init() {

      MAIN_CONTROLLER = new Joystick(RobotMap.MAIN_CONTROLLER.value);
      ARM_CONTROLLER = new Joystick(RobotMap.ARM_CONTROLLER.value);
  
      YButton = new JoystickButton(MAIN_CONTROLLER, 4);
      AButton = new JoystickButton(MAIN_CONTROLLER, 1);
      XButton = new JoystickButton(MAIN_CONTROLLER, 3);
      BButton = new JoystickButton(MAIN_CONTROLLER, 2);
      ArmLeftBumper = new JoystickButton(ARM_CONTROLLER, 5);
      ArmBButton = new JoystickButton(ARM_CONTROLLER, 2);
      drivetrainOverride = new DrivetrainOverride();

      XButton.whenPressed(new DriveForward(10));
      //XButton.whenReleased(new ArcadeDrive()); 
  
      drivetrainOverride.whileActive(new ArcadeDrive());
    }
  
    //Drivebase control
    public double getMainLeftJoyY() {
      double value = MAIN_CONTROLLER.getRawAxis(1);
      return Math.abs(value) > DEADZONE ? ((Math.abs(value) - DEADZONE) * Math.abs(value) / (0.85 * value)) : 0.0;
    }
    
    //For tankdrive control
    public double getMainRightJoyY() {
      double value = MAIN_CONTROLLER.getRawAxis(5);
      return Math.abs(value) > DEADZONE ? ((Math.abs(value) - DEADZONE) * Math.abs(value) / (0.85 * value)) : 0.0;
    }
  
    //Drivebase control
    public double getMainRightJoyX() {
      double value = MAIN_CONTROLLER.getRawAxis(4);
      return Math.abs(value) > DEADZONE ? ((Math.abs(value) - DEADZONE) * Math.abs(value) / (0.85 * value)) : 0.0;
    }
  
    //Drivebase throttle
    public double getMainRightTrigger() {
      double value = MAIN_CONTROLLER.getRawAxis(3);
      return Math.abs(value) > DEADZONE ? ((Math.abs(value) - DEADZONE) * Math.abs(value) / (0.85 * value)) : 0.0;
      //return Math.abs(value) > DEADZONE ? value : 0.0;
    }
  
    //Climber wheel actuation
    public double getMainLeftTrigger() {
      double value = MAIN_CONTROLLER.getRawAxis(2);
      return Math.abs(value) > DEADZONE ? ((Math.abs(value) - DEADZONE) * Math.abs(value) / (0.85 * value)) : 0.0;
      //return Math.abs(value) > DEADZONE ? value : 0.0;
    }
  
    //Elevator control
    public boolean getMainAButton() {
      return MAIN_CONTROLLER.getRawButton(1);
    }

    public boolean getMainBButtonPressed() {
      return MAIN_CONTROLLER.getRawButtonPressed(2);
    }
  
    public boolean getMainLeftBumper() {
      return MAIN_CONTROLLER.getRawButton(5);
    }
    public boolean getMainAButtonPressed() {
      return MAIN_CONTROLLER.getRawButtonPressed(1);
    }
  
    public boolean getMainYButtonPressed() {
      return MAIN_CONTROLLER.getRawButtonPressed(4);
    }
  
    public boolean getMainXButton() {
      return MAIN_CONTROLLER.getRawButton(3);
    }
  
    //toggles which way is "forward" for drivebase
    public boolean getMainRightBumperPressed() {
      return MAIN_CONTROLLER.getRawButtonPressed(6);
    }
  
    //Elevator control
    public boolean getMainYButton() {
      return MAIN_CONTROLLER.getRawButton(4);
    }
  
    //Cargo outtake
    public boolean getArmXButton() {
      return ARM_CONTROLLER.getRawButton(3);
    }
  
    //Cargo intake
    public boolean getArmAButton() {
      return ARM_CONTROLLER.getRawButton(1);
    }
  
    //Hatch control
    public boolean getArmYButton() {
      return ARM_CONTROLLER.getRawButton(4);
    }
  
    
    public boolean getArmBPressed() {
      return ARM_CONTROLLER.getRawButtonPressed(2);
    }
  
    //Arm control
    public double getArmLeftJoyY() {
      double value = ARM_CONTROLLER.getRawAxis(1);
      return Math.abs(value) > DEADZONE ? ((Math.abs(value) - DEADZONE) * Math.abs(value) / (0.85 * value)) : 0.0;
    }
  
    //Arm throttle
    public double getArmRightTrigger() {
      double value = ARM_CONTROLLER.getRawAxis(3);
      return Math.abs(value) > DEADZONE ? ((Math.abs(value) - DEADZONE) * Math.abs(value) / (0.85 * value)) : 0.0;
    }
  
    public double getArmLeftTrigger() {
      double value = ARM_CONTROLLER.getRawAxis(2);
      return Math.abs(value) > DEADZONE ? ((Math.abs(value) - DEADZONE) * Math.abs(value) / (0.85 * value)) : 0.0;
    }
  }
