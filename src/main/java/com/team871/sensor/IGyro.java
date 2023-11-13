package com.team871.sensor;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.CommandBase;

public interface IGyro extends Sendable {

  CommandBase resetGyroCommand();

  double getYaw();
  /**
   * @return the rotation around the Y axis using NEU (north east up) orientation. -180 to 180
   */
  double getPitch();
}
