package com.team871.sensor;

public interface AbsoluteEncoder extends SendableEncoder {
  /**
   * Set the offset to apply to the output.
   */
  void setZeroOffset(double newPosition);
}
