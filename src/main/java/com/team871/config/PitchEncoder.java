package com.team871.config;

import edu.wpi.first.util.sendable.Sendable;

public interface PitchEncoder extends Sendable {
  /**
   * @return the rotation around the Y axis using NEU (north east up) orientation. -180 to 180
   */
  double getPitch();

  void reset();

  double getRawValue();
}
