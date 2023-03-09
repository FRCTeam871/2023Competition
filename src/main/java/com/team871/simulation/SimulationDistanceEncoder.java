package com.team871.simulation;

import com.team871.config.DistanceEncoder;
import edu.wpi.first.util.sendable.SendableBuilder;

public class SimulationDistanceEncoder implements DistanceEncoder {
  private double distance;

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("distance", this::getDistance, this::setDistance);
  }

  @Override
  public double getDistance() {
    return distance;
  }

  public void setDistance(double distance) {
    this.distance = distance;
  }

  @Override
  public void reset() {
    this.distance = 0;
  }

  @Override
  public double getRawValue() {
    // TODO Auto-generated method stub
    return 0;
  }
}
