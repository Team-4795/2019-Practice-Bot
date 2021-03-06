/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */
public enum RobotMap {

    //Controller Mappings
    MAIN_CONTROLLER(0),
    ARM_CONTROLLER(1),
    // PWM Mappings
    SERVO_ONE(8),
    SERVO_TWO(9),
    //CAN Motor Controller Mappings
    LEFT_MOTOR_ONE(1),
    LEFT_MOTOR_TWO(2),
    LEFT_MOTOR_THREE(3),
    RIGHT_MOTOR_ONE(4),
    RIGHT_MOTOR_TWO(5),
    RIGHT_MOTOR_THREE(6),
    ARM_MOTOR(7),
    INTAKE_MOTOR(12),
    CLIMBER_WHEELS(11),
    HATCH_MOTOR(8),
    CLIMBER_MOTOR(20),
    ARM_MOTOR_FOLLOWER(22);
  
    public final int value;
  
    RobotMap(int value) {
      this.value = value;
    }
}