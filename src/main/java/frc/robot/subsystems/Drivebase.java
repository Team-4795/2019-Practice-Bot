/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.ArcadeDrive;

/**
 * Add your docs here.
 */
public class Drivebase extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public final TalonSRX leftMotorOne;
  private final VictorSPX leftMotorTwo;
  private final VictorSPX leftMotorThree;
  public final TalonSRX rightMotorOne;
  private final VictorSPX rightMotorTwo;
  private final VictorSPX rightMotorThree;

  private final double WHEEL_DIAMETER_IN = 8.0;
  private final int ENCODER_COUNTS_PER_REV = 4096;
  public final double ENCODER_COUNTS_PER_FT = 9400;
  private final double kP = 0.0;
  private final double kI = -0.00;
  private final double kD = 0.0;
  private final double kF = .065;
  public final int allowableError = 100;
  //in theory should equal: (ENCODER_COUNTS_PER_REV * 12) / (Math.PI * WHEEL_DIAMETER_IN)
  
  public Drivebase () {


    leftMotorOne = new TalonSRX(RobotMap.LEFT_MOTOR_ONE.value);
    leftMotorTwo = new VictorSPX(RobotMap.LEFT_MOTOR_TWO.value);
    leftMotorThree = new VictorSPX(RobotMap.LEFT_MOTOR_THREE.value);
    rightMotorOne = new TalonSRX(RobotMap.RIGHT_MOTOR_ONE.value);
    rightMotorTwo = new VictorSPX(RobotMap.RIGHT_MOTOR_TWO.value);
    rightMotorThree = new VictorSPX(RobotMap.RIGHT_MOTOR_THREE.value);

    Robot.masterTalon(leftMotorOne);
    Robot.masterTalon(rightMotorOne);

    leftMotorOne.config_kP(0, kP);
    leftMotorOne.config_kI(0, kI);
    leftMotorOne.config_kD(0, kD);
    leftMotorOne.config_kF(0, kF);
    leftMotorOne.configAllowableClosedloopError(0, allowableError, 5000);
    leftMotorOne.configMotionAcceleration(20000);
    leftMotorOne.configMotionCruiseVelocity(15000);

    rightMotorOne.config_kP(0, kP);
    rightMotorOne.config_kI(0, kI);
    rightMotorOne.config_kD(0, kD);
    leftMotorOne.config_kF(0, kF);
    rightMotorOne.configAllowableClosedloopError(0, allowableError, 5000);

    Robot.initVictor(leftMotorTwo);
    Robot.initVictor(leftMotorThree);
    Robot.initVictor(rightMotorTwo);
    Robot.initVictor(rightMotorThree);


    rightMotorOne.setInverted(true);

    //rightMotorTwo.setInverted(true);
    rightMotorThree.setInverted(true);

    //leftMotorTwo.setInverted(true);
    //leftMotorThree.setInverted(true);

    leftMotorTwo.follow(leftMotorOne);
    leftMotorThree.follow(leftMotorOne);

    rightMotorTwo.follow(rightMotorOne);
    rightMotorThree.follow(rightMotorOne);
    
    rightMotorOne.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    leftMotorOne.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    rightMotorOne.setSelectedSensorPosition(0);
    leftMotorOne.setSelectedSensorPosition(0);

  }

  public void setMotors(double left, double right) {
    leftMotorOne.set(ControlMode.PercentOutput, left);
    rightMotorOne.set(ControlMode.PercentOutput, right);
  }

  public void driveFeet(double feet) {
    this.resetEnc();
    leftMotorOne.set(ControlMode.MotionMagic, -feet * ENCODER_COUNTS_PER_FT);
    rightMotorOne.follow(leftMotorOne);
  }

  public double getLeftEnc() {
    return leftMotorOne.getSelectedSensorPosition();
  }

  public double getRightEnc() {
    return rightMotorOne.getSelectedSensorPosition();
  }

  public void resetEnc() {
    leftMotorOne.setSelectedSensorPosition(0);
    rightMotorOne.setSelectedSensorPosition(0);
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new ArcadeDrive());
  }
}

