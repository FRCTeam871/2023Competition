package com.team871.config;

import edu.wpi.first.util.sendable.Sendable;

public interface DistanceEncoder extends Sendable {

  /**
   * @return the distance of the encoder in inches
   */
  double getDistance();

  double getRawValue();

  void reset();
}
