package com.team871.config;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SimulationDualKeyboardControl implements IControlConfig {

    final GenericHID kbd1 = new GenericHID(0);
    final GenericHID kbd2 = new GenericHID(1);

    @Override
    public double getClawAxisValue() {
        return kbd1.getRawAxis(0);
    }

    @Override
    public double getWristAxisValue() {
        return kbd1.getRawAxis(1);
    }

    @Override
    public double getShoulderAxisValue() {
        return kbd1.getRawAxis(2);
    }

    @Override
    public double getExtensionAxisValue() {
        return kbd2.getRawAxis(0);
    }

    @Override
    public double getExtensionAxisTrimValue() {
        return 0;
    }

    @Override
    public double getDriveXAxisValue() {
        return 0;
    }

    @Override
    public double getDriveYAxisValue() {
        return 0;
    }

    @Override
    public double getDriveRotationAxisValue() {
        return 0;
    }

    @Override
    public Trigger getHighNodeTrigger() {
        return new Trigger(
                CommandScheduler.getInstance().getDefaultButtonLoop(), () -> kbd1.getRawButton(1));
    }

    @Override
    public Trigger getMiddleNodeTrigger() {
        return new Trigger(
                CommandScheduler.getInstance().getDefaultButtonLoop(), () -> kbd1.getRawButton(2));
    }

    @Override
    public Trigger getBottomNodeTrigger() {
        return new Trigger(
                CommandScheduler.getInstance().getDefaultButtonLoop(), () -> kbd1.getRawButton(3));
    }

    @Override
    public Trigger getPickupTrigger() {
        return new Trigger();
    }

    @Override
    public Trigger getIntakeTrigger() {
        return new Trigger();
    }

    @Override
    public Trigger getExhaustTrigger() {
        return new Trigger();
    }

    @Override
    public Trigger getBalanceTrigger() {
        return new Trigger();
    }

    @Override
    public Trigger getResetGyroTrigger() {
        return new Trigger();
    }

    @Override
    public Trigger getFoldOutTrigger() {
        return new Trigger();
    }

    @Override
    public Trigger getFoldInTrigger() {
        return new Trigger(
                CommandScheduler.getInstance().getDefaultButtonLoop(), () -> kbd1.getRawButton(4));
    }

    @Override
    public Trigger getHomeExtensionTrigger() {
        return new Trigger(
                CommandScheduler.getInstance().getDefaultButtonLoop(), () -> kbd2.getRawButton(1));
    }

    @Override
    public Trigger getManualControl() {
        return new Trigger(
                CommandScheduler.getInstance().getDefaultButtonLoop(), () -> kbd2.getRawButton(2));
    }
}
