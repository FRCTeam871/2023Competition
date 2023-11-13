package com.team871.sensor;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.util.sendable.SendableBuilder;

public class SRXIncrementalEncoder implements IncrementalEncoder {

  private final WPI_TalonSRX talon;
  private double inchesPerTick;

  public SRXIncrementalEncoder(final WPI_TalonSRX talonSRX, final double inchesPerTick) {
    this.talon = talonSRX;
    this.inchesPerTick = inchesPerTick;
  }

  @Override
  public double getPosition() {
    return talon.getSelectedSensorPosition() * inchesPerTick;
  }

  @Override
  public void setPosition(double position) {
    talon.getSensorCollection().setQuadraturePosition((int)position, 0);
  }

  @Override
  public double getRawValue() {
    return talon.getSelectedSensorPosition();
  }

  @Override
  public void setScaleFactor(double kScale) {
      this.inchesPerTick = kScale;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Position", this::getPosition, null);
    builder.addDoubleProperty("Raw", this::getRawValue, null);
  }
}
