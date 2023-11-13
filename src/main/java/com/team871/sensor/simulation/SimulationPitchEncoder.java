package com.team871.sensor.simulation;

import com.team871.sensor.AbsoluteEncoder;
import edu.wpi.first.util.sendable.SendableBuilder;

public class SimulationPitchEncoder implements AbsoluteEncoder {
  private double pitch;

  public double getPosition() {
    return pitch;
  }

  public void setPitch(double pitch) {
    this.pitch = pitch;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("pitch", this::getPosition, this::setPitch);
  }

  @Override
  public void reset() {
    this.pitch = 0;
  }

  @Override
  public double getRawValue() {
    // TODO Auto-generated method stub
    return 0;
  }
}
