package com.team871.sensor;

import edu.wpi.first.util.sendable.Sendable;

/**
 * An encoder that is also a sendable object to hide actual hardware implementaiton.
 */
public interface SendableEncoder extends Sendable {

  /**
   * Get the incremental position of the encoder.  The units of this value
   * depend on the scale factor provided.
   * @return
   */
  double getPosition();

  /**
   * Get the raw count of the encoder.  Note that for some implementations (ie CANCoder or SparkMax built in)
   * the scale value is applied directly by the sensor itself, so this might look identical to {@link #getPosition()}
   * @return
   */
  double getRawValue();

  /**
   * Set the scale factor to apply to the output. Output = kScale * rawValue
   * @param kScale the scale factor
   */
  void setScaleFactor(double kScale);
}
