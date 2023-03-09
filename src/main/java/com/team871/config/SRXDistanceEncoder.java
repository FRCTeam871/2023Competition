package com.team871.config;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.util.sendable.SendableBuilder;

public class SRXDistanceEncoder implements DistanceEncoder {

  private final WPI_TalonSRX talon;
  private double inchesPerTick;

  public SRXDistanceEncoder(WPI_TalonSRX talonSRX, double inchesPerTick) {

    this.talon = talonSRX;
    this.inchesPerTick = inchesPerTick;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("distance", this::getDistance, null);
    builder.addDoubleProperty("rawInput", this::getRawValue, null);
  }

  @Override
  public double getDistance() {
    return talon.getSelectedSensorPosition() * inchesPerTick;
  }

  @Override
  public void reset() {
    talon.getSensorCollection().setQuadraturePosition(0, 0);
  }

  @Override
  public double getRawValue() {
    return talon.getSelectedSensorPosition();
  }
}
