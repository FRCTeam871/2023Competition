package com.team871.subsystems;

import com.team871.config.DistanceEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ArmExtension extends PIDSubsystem {

  private static final double EXTENSION_PID_KP = 1;
  private static final double EXTENSION_PID_KI = 0;
  private static final double EXTENSION_PID_KD = 0;

  private final MotorController extensionMotor;
  private final DistanceEncoder distanceEncoder;

  private boolean isHomed = false;

  public ArmExtension(final MotorController extensionMotor, final DistanceEncoder distanceEncoder) {
    super(new PIDController(EXTENSION_PID_KP, EXTENSION_PID_KI, EXTENSION_PID_KD));
    this.extensionMotor = extensionMotor;
    this.distanceEncoder = distanceEncoder;
    getController().setTolerance(0.05);

    SmartDashboard.putData("extensionPID", getController());
    SmartDashboard.putData("extensionEncoder", distanceEncoder);
    SmartDashboard.putData("resetCommand", resetExtensionEncoderCommand());
  }

  static double limitOutput(double rawInput, double currentDistance) {
    if (rawInput < 0) {
      double maxoutput = Math.max(currentDistance / 3, .3);
      return Math.max(-maxoutput, rawInput);
    } else {
      double maxoutput = Math.max((19 - currentDistance) / 3, .3);
      return Math.min(maxoutput, rawInput);
    }
  }

  /**
   * @param output retract is negative, extend is positive, output between -1 and 1
   */
  public void moveExtension(final double output) {
    final double limitedOutput = limitOutput(output, distanceEncoder.getDistance());
    extensionMotor.set(limitedOutput);
    SmartDashboard.putNumber("extensionMotorOutput", limitedOutput);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }

  public void manualExtensionCommand(DoubleSupplier extension) {
    final Command command = run(() -> moveExtension(extension.getAsDouble()));
    command.setName("ManualExtensionCommand");
    setDefaultCommand(command);
  }

  public CommandBase resetExtensionEncoderCommand() {
    return runOnce(distanceEncoder::reset);
  }

  public CommandBase homeExtensionCommand(final BooleanSupplier isAtLimit) {
    return run(() -> moveExtension(-.5)).until(isAtLimit);
  }

  @Override
  protected void useOutput(double output, double setpoint) {
    moveExtension(output);
  }

  @Override
  protected double getMeasurement() {
    return distanceEncoder.getDistance();
  }

  public boolean isAtSetpoint() {
    final boolean atSetpoint = getController().atSetpoint();
    if (atSetpoint) {
      isHomed = true;
    }
    return atSetpoint;
  }

  @Override
  public void setSetpoint(double setpoint) {
    // Explicitly don't let us go past 0, or beyond 18
    super.setSetpoint(Math.min(18.5, Math.max(0, setpoint)));
  }

  public boolean isHomed() {
    return isHomed;
  }

  public Command run(String name, Runnable action) {
    final CommandBase actual = run(action);
    actual.setName(name);
    return actual;
  }
}
