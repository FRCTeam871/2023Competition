package com.team871.sensor;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.util.sendable.SendableBuilder;

public class SRXAnalogEncoder implements AbsoluteEncoder {

  private final WPI_TalonSRX talon;
  private double zeroOffset;
  private double degreesPerTick;

  public SRXAnalogEncoder(WPI_TalonSRX talonSRX, double zeroOffset, double degreesPerTick) {

    this.talon = talonSRX;
    this.zeroOffset = zeroOffset;
    this.degreesPerTick = degreesPerTick;
  }
  // this is a test method
  public double calculateDegrees(double rawInput) {
    return (rawInput - zeroOffset) * degreesPerTick;
  }

  @Override
  public double getPosition() {
    return calculateDegrees(talon.getSelectedSensorPosition());
  }

  @Override
  public double getRawValue() {
    return talon.getSelectedSensorPosition();
  }

  @Override
  public void setZeroOffset(final double position) {
    talon.getSensorCollection().setAnalogPosition((int)position, 0);
  }

  @Override
  public void setScaleFactor(double kScale) {
      this.degreesPerTick = kScale;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Position", this::getPosition, null);
    builder.addDoubleProperty("Raw", this::getRawValue, null);
  }
}
