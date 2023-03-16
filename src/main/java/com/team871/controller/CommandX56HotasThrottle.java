package com.team871.controller;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandX56HotasThrottle {
  // Axes
  private static final int LEFT_THROTTLE_AXIS = 0;
  private static final int RIGHT_THROTTLE_AXIS = 1;
  private static final int F_AXIS = 2;
  private static final int THUMB_STICK_X_AXIS = 3;
  private static final int THUMB_STICK_Y_AXIS = 4;

  private static final int G_AXIS = 5;
  private static final int ROTARY_3 = 6;
  private static final int ROTARY_4 = 7;

  // Buttons
  private static final int E_BUTTON = 1;
  private static final int F_BUTTON = 2;
  private static final int G_BUTTON = 3;
  private static final int I_BUTTON = 4;
  private static final int H_BUTTON = 5;
  private static final int SW_1 = 6;
  private static final int SW_6 = 11;

  private static final int TGL_1_UP = 12;

  private final GenericHID hid;

  public CommandX56HotasThrottle(final int port) {
    this.hid = new GenericHID(port);
  }

  /**
   * Get the left throttle axis. -1 is fully forward, +1 is fully back
   *
   * @return the left throttle value
   */
  public double getLeftThrottle() {
    return hid.getRawAxis(LEFT_THROTTLE_AXIS);
  }

  /**
   * Get the right throttle axis. -1 is fully forward, +1 is fully back
   *
   * @return the left throttle value
   */
  public double getRightThrottle() {
    return hid.getRawAxis(RIGHT_THROTTLE_AXIS);
  }

  public double getFAxis() {
    return hid.getRawAxis(F_AXIS);
  }

  public double getGAxis() {
    return hid.getRawAxis(G_AXIS);
  }

  public double getThumbStickX() {
    return hid.getRawAxis(THUMB_STICK_X_AXIS);
  }

  public double getThumbStickY() {
    return hid.getRawAxis(THUMB_STICK_Y_AXIS);
  }

  public double getRotary3() {
    SmartDashboard.putNumber("rotary3", hid.getRawAxis(ROTARY_3));
    return hid.getRawAxis(ROTARY_3);
  }

  public double getRotary4() {
    SmartDashboard.putNumber("rotary4", hid.getRawAxis(ROTARY_4));
    return hid.getRawAxis(ROTARY_4);
  }

  public Trigger getSw(final int swNumber) {
    if (swNumber < 1 || swNumber > 6) {
      throw new IllegalArgumentException("No such SW switch " + swNumber);
    }

    // The SW switches are all sequential, so we can compute it instead of having a hozillion
    // getters
    return new Trigger(
        CommandScheduler.getInstance().getDefaultButtonLoop(),
        () -> hid.getRawButton((swNumber - 1) + SW_1));
  }

  public Trigger getToggleUp(int toggleNum) {
    if (toggleNum < 1 || toggleNum > 4) {
      throw new IllegalArgumentException("No such TGL switch " + toggleNum);
    }
    return new Trigger(
        CommandScheduler.getInstance().getDefaultButtonLoop(),
        () -> hid.getRawButton(((toggleNum - 1) * 2) + TGL_1_UP));
  }

  public Trigger getToggleDown(int toggleNum) {
    if (toggleNum < 1 || toggleNum > 4) {
      throw new IllegalArgumentException("No such TGL switch " + toggleNum);
    }
    return new Trigger(
        CommandScheduler.getInstance().getDefaultButtonLoop(),
        () -> hid.getRawButton(((toggleNum - 1) * 2) + TGL_1_UP + 1));
  }

  public Trigger getEButton() {
    return new Trigger(
        CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getRawButton(E_BUTTON));
  }

  public Trigger getFButton() {
    return new Trigger(
        CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getRawButton(F_BUTTON));
  }

  public Trigger getGButton() {
    return new Trigger(
        CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getRawButton(G_BUTTON));
  }

  public Trigger getHButton() {
    return new Trigger(
        CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getRawButton(H_BUTTON));
  }

  public Trigger getIButton() {
    return new Trigger(
        CommandScheduler.getInstance().getDefaultButtonLoop(), () -> hid.getRawButton(I_BUTTON));
  }
}
