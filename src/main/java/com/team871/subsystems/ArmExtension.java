package com.team871.subsystems;

import com.team871.config.DistanceEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;

public class ArmExtension extends SubsystemBase {

  private static final double EXTENSION_PID_KP = 0.00012207;
  private static final double EXTENSION_PID_KI = 0;
  private static final double EXTENSION_PID_KD = 0;

  private final MotorController extensionMotor;
  private final PIDController extensionPID;
  private final DistanceEncoder distanceEncoder;
  private double positionInchesSetpoint;


  public ArmExtension(final MotorController extensionMotor, final DistanceEncoder distanceEncoder) {
    this.extensionMotor = extensionMotor;
    extensionPID = new PIDController(EXTENSION_PID_KP, EXTENSION_PID_KI, EXTENSION_PID_KD);
    this.distanceEncoder = distanceEncoder;
  }

  public void moveExtension(final double output) {
    SmartDashboard.putNumber("extensionMotorOutput", output);
    extensionMotor.set(output);
  }

  public CommandBase moveExtensionCommand(double output) {
    return run(() -> moveExtension(output));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("distanceSetpoint", this::getPositionInchesSetpoint, this::setPositionInchesSetpoint);
    SmartDashboard.putData("extensionPID", extensionPID);
  }

  public void setdefaultCommand(Supplier<Double> extension) {
    setDefaultCommand(
        run(
            () -> {
              moveExtension(extension.get());
            }));
  }

  public CommandBase extensionPIDCommand() {
    return new PIDCommand(
            extensionPID,
            distanceEncoder::getDistance,
            this::getPositionInchesSetpoint,
            this::moveExtension, this);
  }

  public CommandBase resetExtensionEncoderCommand() {
    return runOnce(distanceEncoder::reset);
  }

  public double getPositionInchesSetpoint() {
    return positionInchesSetpoint;
  }

  public void setPositionInchesSetpoint(double positionInchesSetpoint) {
    this.positionInchesSetpoint = positionInchesSetpoint;
  }
}