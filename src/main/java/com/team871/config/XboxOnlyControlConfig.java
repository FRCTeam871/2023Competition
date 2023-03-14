package com.team871.config;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxOnlyControlConfig implements IControlConfig {

    private static final double LEFT_X_DEADBAND = .09;
    private static final double LEFT_Y_DEADBAND = .09;
    private static final double RIGHT_X_DEADBAND = .09;
    private static final double RIGHT_Y_DEADBAND = .09;

    private static final double TRIGGER_DEADBAND = 0.01;

    private final CommandXboxController driveController;
    private final CommandXboxController systemController;

    public XboxOnlyControlConfig() {
        driveController = new CommandXboxController(0);
        systemController = new CommandXboxController(1);
    }

    @Override
    public double getClawAxisValue() {
        return getCompoundTriggerAxis(driveController);
    }

    @Override
    public double getWristAxisValue() {
        return MathUtil.applyDeadband(systemController.getRightY(), RIGHT_Y_DEADBAND);
    }

    @Override
    public double getShoulderAxisValue() {
        return MathUtil.applyDeadband(systemController.getLeftY(), LEFT_Y_DEADBAND);
    }

    @Override
    public double getExtensionAxisValue() {
        return getCompoundTriggerAxis(systemController);
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
        return systemController.y();
    }

    @Override
    public Trigger getMiddleNodeTrigger() {
        return systemController.b();
    }

    @Override
    public Trigger getBottomNodeTrigger() {
        return systemController.a();
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
        final double compoundAxis = -controller.getLeftTriggerAxis() + controller.getRightTriggerAxis();
        return MathUtil.applyDeadband(compoundAxis, TRIGGER_DEADBAND);
    }

    @Override
    public Trigger getFoldOutTrigger() {
        return systemController.start();
    }

    @Override
    public Trigger getFoldInTrigger() {
        return systemController.back();
    }

    @Override
    public Trigger getHomeExtensionTrigger() {
        return driveController.x();
    }

    @Override
    public Trigger getPickupTrigger() {
        // TODO Auto-generated method stub
        return systemController.x();
    }
}
