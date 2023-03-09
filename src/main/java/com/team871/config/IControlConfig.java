package com.team871.config;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This interface contains the mappings between various buttons and axes to their functions.
 */
public interface IControlConfig {
    double getClawAxisValue();

    double getWristAxisValue();

    double getShoulderAxisValue();

    double getExtensionAxisValue();

    double getDriveXAxisValue();

    double getDriveYAxisValue();

    double getDriveRotationAxisValue();

    Trigger getHighNodeTrigger();

    Trigger getMiddleNodeTrigger();

    Trigger getBottomNodeTrigger();

    Trigger getIntakeTrigger();

    Trigger getExhaustTrigger();

    Trigger getBalanceTrigger();

    Trigger getResetGyroTrigger();
}
