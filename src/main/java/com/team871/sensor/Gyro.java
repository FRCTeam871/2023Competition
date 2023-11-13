package com.team871.sensor;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public class Gyro implements IGyro {
  private final AHRS gyro = new AHRS();

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Pitch", this::getPitch, null);
    builder.addDoubleProperty("Yaw", this::getYaw, null);
    SmartDashboard.putData("resetGyro", resetGyroCommand());
  }

  public CommandBase resetGyroCommand() {
    return Commands.runOnce(() -> gyro.reset());
  }

  public double getYaw() {
    return gyro.getYaw();
  }

  /**
   * @return the rotation around the Y axis using NEU (north east up) orientation. -180 to 180
   */
  public double getPitch() {
    return -1 * gyro.getRoll();
  }
}
