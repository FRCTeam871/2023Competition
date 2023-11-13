package com.team871.sensor.simulation;

import com.team871.sensor.SendableEncoder;
import edu.wpi.first.util.sendable.SendableBuilder;

public class SimulationDistanceEncoder implements SendableEncoder {
  private double distance;

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("distance", this::getPosition, this::setDistance);
  }

  @Override
  public double getPosition() {
    return distance;
  }

  public void setDistance(double distance) {
    this.distance = distance;
  }

  @Override
  public void setPosition() {
    this.distance = 0;
  }

  @Override
  public double getRawValue() {
    // TODO Auto-generated method stub
    return 0;
  }
}
