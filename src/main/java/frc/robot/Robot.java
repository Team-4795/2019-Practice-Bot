/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.buttons.Trigger;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DriveForward;
import frc.robot.commands.ManualHatchControl;
import frc.robot.subsystems.Hatch;
import frc.robot.triggers.DrivetrainOverride;
import frc.robot.subsystems.Drivebase;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {

  public static Drivebase drivebase;
  public static OI oi;
  public static Hatch hatch;
  private UsbCamera hatchCam;

  @Override
  public void robotInit() {
    drivebase = new Drivebase();
    oi = new OI();
    oi.init();
    hatch = new Hatch();
    hatchCam = CameraServer.getInstance().startAutomaticCapture(0);
    CameraServer.getInstance().addCamera(hatchCam);

    /*hatchCam.setVideoMode(PixelFormat.kRGB565, 360, 240, 25);
    //SmartDashboard.putBoolean("set hatch res", Robot.hatchCam.setResolution(4, 3));
    //Robot.hatchCam.setResolution(16, 12);
    hatchCam.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    hatchCam.setExposureHoldCurrent();
    hatchCam.setWhiteBalanceHoldCurrent();
    Robot.switcher.setSource(Robot.hatchCam);

    CameraServer.getInstance().startAutomaticCapture(0);*/

  }

  @Override
  public void disabledInit() {
    Robot.hatch.servoUp = true;
    Robot.hatch.hatchUp = false;
  }

  @Override
  public void autonomousInit() {
    //Scheduler.getInstance().add(new DriveForward(0.4));;
  }

  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }
  
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    //SmartDashboard.putNumber("LeftJoystick Y Axis", Robot.oi.getMainLeftJoyY());
    //SmartDashboard.putNumber("RightJoystick X Axis", Robot.oi.getMainRightJoyX());
    //SmartDashboard.putBoolean("A Button", Robot.oi.getMainAButton());
    SmartDashboard.putNumber("Left Encoder", Robot.drivebase.getLeftEnc());
    SmartDashboard.putNumber("Right Encoder", Robot.drivebase.getRightEnc());
  }

  public static void masterTalon(TalonSRX motor) {
    motor.setNeutralMode(NeutralMode.Brake);
    motor.configContinuousCurrentLimit(10, 0);
    motor.configPeakCurrentLimit(12, 0);
    motor.configPeakCurrentDuration(20, 0);
    motor.enableCurrentLimit(true);
    motor.configOpenloopRamp(0.2, 0);
    motor.configClosedloopRamp(0.2, 0);
  }

  public static void initTalon(TalonSRX motor) {
    motor.setNeutralMode(NeutralMode.Brake);
    motor.neutralOutput();
    motor.setSensorPhase(false);
    motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    motor.configNominalOutputForward(0.0, 0);
    motor.configNominalOutputReverse(0.0, 0);
  }

  public static void initVictor(VictorSPX motor) {
    motor.setNeutralMode(NeutralMode.Brake);
    motor.neutralOutput();
    motor.setSensorPhase(false);
    motor.configNominalOutputForward(0.0, 0);
    motor.configNominalOutputReverse(0.0, 0);
  }
}
