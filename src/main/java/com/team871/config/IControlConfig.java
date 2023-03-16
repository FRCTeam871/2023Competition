package com.team871.config;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/** This interface contains the mappings between various buttons and axes to their functions. */
public interface IControlConfig {
  double getClawAxisValue();

  double getWristAxisValue();

  double getShoulderAxisValue();

  double getExtensionAxisValue();

  double getExtensionAxisTrimValue();

  double getDriveXAxisValue();

  double getDriveYAxisValue();

  double getDriveRotationAxisValue();

  Trigger getHighNodeTrigger();

  Trigger getMiddleNodeTrigger();

  Trigger getBottomNodeTrigger();

  Trigger getPickupTrigger();

  Trigger getIntakeTrigger();

  Trigger getExhaustTrigger();

  Trigger getBalanceTrigger();

  Trigger getResetGyroTrigger();

  Trigger getFoldOutTrigger();

  Trigger getFoldInTrigger();

  Trigger getHomeExtensionTrigger();

  Trigger getManualControl();
}
