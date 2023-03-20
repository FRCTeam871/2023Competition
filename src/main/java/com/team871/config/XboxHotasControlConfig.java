package com.team871.config;

import com.team871.controller.CommandX56HotasThrottle;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxHotasControlConfig implements IControlConfig {

  private static final double LEFT_X_DEADBAND = .09;
  private static final double LEFT_Y_DEADBAND = .09;
  private static final double RIGHT_X_DEADBAND = .09;
  private static final double RIGHT_Y_DEADBAND = .09;

  private static final double TRIGGER_DEADBAND = 0.01;

  private final CommandXboxController driveController;
  private final CommandX56HotasThrottle systemController;

  public XboxHotasControlConfig() {
    driveController = new CommandXboxController(0);
    systemController = new CommandX56HotasThrottle(1);
  }

  @Override
  public double getClawAxisValue() {
    return getCompoundTriggerAxis(driveController);
  }

  @Override
  public double getWristAxisValue() {
    return -systemController.getRightThrottle();
  }

  @Override
  public double getShoulderAxisValue() {
    return systemController.getRotary3();
  }

  @Override
  public double getExtensionAxisValue() {
    // The throttle is negative when fully forward and positive when fully back.
    // This normalizes the throttle so that we get a 0 - 1 value where
    // 0 is fully back, and 1 is fully forward
    return (-systemController.getLeftThrottle() + 1.0d)/2.0d;
  }

  @Override
  public double getExtensionAxisTrimValue() {
    return -systemController.getLeftThrottle();
  }

  @Override
  public double getDriveXAxisValue() {
    return -driveController.getLeftY();
  }

  @Override
  public double getDriveYAxisValue() {
    return driveController.getLeftX();
  }

  @Override
  public double getDriveRotationAxisValue() {
    return driveController.getRightX();
  }

  @Override
  public Trigger getHighNodeTrigger() {
    return driveController.y();
  }

  @Override
  public Trigger getMiddleNodeTrigger() {
    return driveController.b();
  }

  @Override
  public Trigger getBottomNodeTrigger() {
    return driveController.a();
  }

  @Override
  public Trigger getIntakeTrigger() {
    return driveController.rightBumper();
  }

  @Override
  public Trigger getExhaustTrigger() {
    return driveController.leftBumper();
  }

  @Override
  public Trigger getBalanceTrigger() {
    return driveController.b();
  }

  @Override
  public Trigger getResetGyroTrigger() {
    return driveController.start();
  }

  private double getCompoundTriggerAxis(final CommandXboxController controller) {
    // The left and right trigger axes are actually the same axes.  The left trigger will make the
    // axes go from 0
    // to -1,  while the right will go from 0 to + 1.  We can simply add the two together to get a
    // single compound
    // axis that looks like one. Notably, if the driver pulls them both, they cancel out to 0.
    final double compoundAxis = -controller.getLeftTriggerAxis() + controller.getRightTriggerAxis();
    return MathUtil.applyDeadband(compoundAxis, TRIGGER_DEADBAND);
  }

  @Override
  public Trigger getFoldOutTrigger() {
    return driveController.a();
  }

  @Override
  public Trigger getFoldInTrigger() {
    return driveController.x();
  }

  @Override
  public Trigger getHomeExtensionTrigger() {
    // TODO Auto-generated method stub
    return systemController.getToggleUp(1);
  }

  @Override
  public Trigger getPickupTrigger() {
    return systemController.getSw(6);
  }

  @Override
  public Trigger getManualControl() {
    // TODO Auto-generated method stub
    return systemController.getToggleDown(2);
  }
}
