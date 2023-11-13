package com.team871.config.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import java.util.function.BooleanSupplier;
import com.team871.sensor.SendableEncoder;
import com.team871.sensor.IGyro;
import com.team871.sensor.AbsoluteEncoder;

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

  SendableEncoder getExtensionEncoder();

  AbsoluteEncoder getWristPitchEncoder();

  AbsoluteEncoder getShoulderPitchEncoder();

  double getMaxWristTrimOffset();

  double getMaxShoulderTrimOffset();

  double getMaxExtensionTrimOffset();

  double getTopShoulderSetpoint();

  double getMiddleShoulderSetpoint();

  double getBottomShoulderSetpoint();

  double getTopExtensionSetpoint();

  double getMiddleExtensionSetpoint();

  double getBottomExtensionSetpoint();

  double getShoulderLowClampValue();

  double getShoulderHighClampValue();

  double getFoldInShoulderSetpoint();

  double getFoldInExtensionSetpoint();

  BooleanSupplier getIsExtensionRetracted();
}
