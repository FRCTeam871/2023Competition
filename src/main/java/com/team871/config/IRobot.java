package com.team871.config;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public interface IRobot {
  /**
   * @return front left motor
   */
  MotorController getFrontLeftMotor();

  /**
   * @return rear left motor
   */
  MotorController getRearLeftMotor();

  /**
   * @return front right motor
   */
  MotorController getFrontRightMotor();

  /**
   * @return rear right motor
   */
  MotorController getRearRightMotor();

  MotorController getShoulderMotor();

  MotorController getArmExtensionMotor();

  MotorController getWristMotor();

  MotorController getClawMotor();

  MotorController getLeftIntakeMotor();

  MotorController getRightIntakeMotor();

  /**
   * @return gyro
   */
  IGyro gyro();

  PIDController getBalancePID();

  DistanceEncoder getExtensionEncoder();

  PitchEncoder getWristPitchEncoder();

  PitchEncoder getShoulderPitchEncoder();

  double getMaxOffsetWristValue();

  double getMaxOffsetShoulderValue();

  double getTopShoulderSetpoint();

  double getMiddleShoulderSetpoint();

  double getBottomShoulderSetpoint();

  double getTopExtensionSetpoint();

  double getMiddleExtensionSetpoint();

  double getBottomExtensionSetpoint();

  double getPickupExtensionSetpoint();

  double getRestOnFrameSetpoint();

  double getShoulderLowClampValue();

  double getShoulderHighClampValue();

  double getFoldInShoulderSetpoint();

  double getFoldInExtensionSetpoint();

  double getFoldOutShoulderSetpoint();

  double getFoldOutExtensionSetpoint();

  BooleanSupplier getIsExtensionRetracted();
}
