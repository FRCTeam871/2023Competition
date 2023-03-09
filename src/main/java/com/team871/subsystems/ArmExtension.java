package com.team871.subsystems;

import com.team871.config.DistanceEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class ArmExtension extends SubsystemBase {

  private static final double EXTENSION_PID_KP = 0.45;
  private static final double EXTENSION_PID_KI = 0;
  private static final double EXTENSION_PID_KD = 0;

  private final MotorController extensionMotor;
  private final PIDController extensionPID;
  private final DistanceEncoder distanceEncoder;

  public ArmExtension(final MotorController extensionMotor, final DistanceEncoder distanceEncoder) {
    this.extensionMotor = extensionMotor;
    this.extensionPID = new PIDController(EXTENSION_PID_KP, EXTENSION_PID_KI, EXTENSION_PID_KD);
    extensionPID.setTolerance(0.5);
    this.distanceEncoder = distanceEncoder;
    
    SmartDashboard.putData("extensionPID", extensionPID);
    // SmartDashboard.putData("ExtensionPIDCommand", extensionPIDCommand());
    SmartDashboard.putData("extensionEncoder", distanceEncoder);
    SmartDashboard.putData("resetCommand", resetExtensionEncoderCommand());
  }

  static double limitOutput(double rawInput, double currentDistance) {
    if (rawInput < 0) {
      double maxoutput = Math.max(currentDistance / 3, .3);
      return Math.max(-maxoutput, rawInput);
    } else {
      double maxoutput = Math.max((19 - currentDistance)/3, .3);
      return Math.min(maxoutput, rawInput);
    }
  }

  /**
   * @param output retract is negative, extend is positive, output between -1 and 1
   */
  public void moveExtension(final double output) {
    double limitedOutput = limitOutput(output, distanceEncoder.getDistance());
    SmartDashboard.putNumber("extensionMotorOutput", limitedOutput);
    extensionMotor.set(limitedOutput);
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

  public CommandBase extensionPIDCommand(String name, DoubleSupplier setpointSupplier) {
    final CommandBase command =
        new PIDCommand(
            extensionPID,
            distanceEncoder::getDistance,
            setpointSupplier,
            this::moveExtension,
            this);

    command.setName(name);
    return command;
  }

  public CommandBase resetExtensionEncoderCommand() {
    return runOnce(distanceEncoder::reset);
  }

  public boolean isAtSetpoint() {
    return extensionPID.atSetpoint();
  }


}