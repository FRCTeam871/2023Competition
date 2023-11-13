package com.team871.sensor.simulation;

import com.team871.sensor.IGyro;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public class SimulationGyro implements IGyro {
  private double yaw = 0;
  private double pitch = 0;

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Pitch", this::getPitch, this::setPitch);
    builder.addDoubleProperty("Yaw", this::getYaw, this::setYaw);
    SmartDashboard.putData("resetGyro", resetGyroCommand());
  }

  public CommandBase resetGyroCommand() {
    return Commands.runOnce(
        () -> {
          this.pitch = 0;
          this.yaw = 0;
        });
  }

  public double getYaw() {
    return yaw;
  }

  public void setYaw(double yaw) {
    this.yaw = yaw;
  }

  public double getPitch() {
    return pitch;
  }

  public void setPitch(double pitch) {
    this.pitch = pitch;
  }
}
