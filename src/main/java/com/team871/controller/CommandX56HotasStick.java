package com.team871.controller;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandX56HotasStick {
    // Axes
    private static final int X_AXIS = 0;
    private static final int Y_AXIS = 1;
    private static final int THUMB_STICK_X_AXIS = 2;
    private static final int THUMB_STICK_Y_AXIS = 3;
    private static final int TWIST_AXIS = 4;

    private static final int TRIGGER = 1;
    private static final int A = 2;
    private static final int B = 3;
    private static final int C = 4;
    private static final int D = 5;
    private static final int PINKIE = 6;
    private static final int H1_UP = 7;
    private static final int H1_RIGHT = 8;
    private static final int H1_DOWN = 9;
    private static final int H1_LEFT = 10;

    private static final int H2_UP = 11;
    private static final int H2_RIGHT = 12;
    private static final int H2_DOWN = 13;
    private static final int H2_LEFT = 14;


    private final GenericHID hid;

    public CommandX56HotasStick(final int port) {
        this.hid = new GenericHID(port);
    }

    /**
     * Get the left throttle axis. -1 is fully forward, +1 is fully back
     * @return the left throttle value
     */
    public double getStickX() {
        return hid.getRawAxis(X_AXIS);
    }

    /**
     * Get the right throttle axis. -1 is fully forward, +1 is fully back
     * @return the left throttle value
     */
    public double getStickY() {
        return hid.getRawAxis(Y_AXIS);
    }

    public double getRotation() {
        return hid.getRawAxis(TWIST_AXIS);
    }

    public double getThumbStickX() {
        return hid.getRawAxis(THUMB_STICK_X_AXIS);
    }

    public double getThumbStickY() {
        return hid.getRawAxis(THUMB_STICK_Y_AXIS);
    }

    public Trigger a() {
        return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(),
                () -> hid.getRawButton(A));
    }

    public Trigger b() {
        return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(),
                () -> hid.getRawButton(B));
    }

    public Trigger c() {
        return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(),
                () -> hid.getRawButton(C));
    }

    public Trigger d() {
        return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(),
                () -> hid.getRawButton(D));
    }
}
