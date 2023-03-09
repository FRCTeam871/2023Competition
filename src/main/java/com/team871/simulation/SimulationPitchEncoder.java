package com.team871.simulation;

import com.team871.config.PitchEncoder;
import edu.wpi.first.util.sendable.SendableBuilder;

public class SimulationPitchEncoder implements PitchEncoder {
  private double pitch;

  public double getPitch() {
    return pitch;
  }

  public void setPitch(double pitch) {
    this.pitch = pitch;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("pitch", this::getPitch, this::setPitch);
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
