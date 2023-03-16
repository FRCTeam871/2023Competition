package com.team871.config;

import com.team871.controller.CommandX56HotasStick;
import com.team871.controller.CommandX56HotasThrottle;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class HotasOnlyControlConfig implements IControlConfig {
    private static final double TRIGGER_DEADBAND = 0.01;

    private final CommandX56HotasStick driveController;
    private final CommandX56HotasThrottle systemController;

    public HotasOnlyControlConfig() {
        driveController = new CommandX56HotasStick(0);
        systemController = new CommandX56HotasThrottle(1);
    }

    @Override
    public double getClawAxisValue() {
        return driveController.getThumbStickY();
    }

    @Override
    public double getWristAxisValue() {
        return systemController.getFAxis();
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
        return 0;
    }

    @Override
    public double getDriveXAxisValue() {
        return driveController.getStickY();
    }

    @Override
    public double getDriveYAxisValue() {
        return driveController.getStickX();
    }

    @Override
    public double getDriveRotationAxisValue() {
        return driveController.getRotation();
    }

    @Override
    public Trigger getHighNodeTrigger() {
        return systemController.getSw(1);
    }

    @Override
    public Trigger getMiddleNodeTrigger() {
        return systemController.getSw(3);
    }

    @Override
    public Trigger getBottomNodeTrigger() {
        return systemController.getSw(5);
    }

    @Override
    public Trigger getPickupTrigger() {
        return systemController.getSw(6);
    }

    @Override
    public Trigger getIntakeTrigger() {
        return driveController.trigger();
    }

    @Override
    public Trigger getExhaustTrigger() {
        return driveController.d();
    }

    @Override
    public Trigger getBalanceTrigger() {
        return driveController.b();
    }

    @Override
    public Trigger getResetGyroTrigger() {
        return driveController.a();
    }

    @Override
    public Trigger getFoldOutTrigger() {
     return systemController.getSw(2);
    }

    @Override
    public Trigger getFoldInTrigger() {
        return systemController.getSw(4);
    }

    @Override
    public Trigger getHomeExtensionTrigger() {
        return systemController.getToggleUp(1);
    }

    @Override
    public Trigger getManualControl() {
        return systemController.getToggleDown(1);
    }
}
