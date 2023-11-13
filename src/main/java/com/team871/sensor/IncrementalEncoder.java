package com.team871.sensor;

public interface IncrementalEncoder extends SendableEncoder {
  /**
   * Set the position of the encoder.
   */
  void setPosition(double newPosition);
}
