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
        return systemController.getFAxis();
    }

    @Override
    public double getShoulderAxisValue() {
        return systemController.getGAxis();
    }

    @Override
    public double getExtensionAxisValue() {
        return systemController.getLeftThrottle();
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
        // The left and right trigger axes are actually the same axes.  The left trigger will make the axes go from 0
        // to -1,  while the right will go from 0 to + 1.  We can simply add the two together to get a single compound
        // axis that looks like one. Notably, if the driver pulls them both, they cancel out to 0.
        final double compoundAxis = controller.getLeftTriggerAxis() + controller.getRightTriggerAxis();
        return MathUtil.applyDeadband(compoundAxis, TRIGGER_DEADBAND);
    }
}
