package com.team871.sensor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxAnalogSensor.Mode;
import edu.wpi.first.util.sendable.SendableBuilder;

public class SparkMaxAnalogEncoder implements AbsoluteEncoder {

  private final CANSparkMax controller;
  private double zeroOffset;

  public SparkMaxAnalogEncoder(final CANSparkMax CANSparkMaxx, final double zeroOffset, final double degreesPerVolt) {
    this.controller = CANSparkMaxx;
    this.zeroOffset = zeroOffset;
    setScaleFactor(degreesPerVolt);
  }

  @Override
  public double getPosition() {
    final double position = getSensor().getPosition();
    return position - zeroOffset;
  }

  @Override
  public double getRawValue() {
    return getSensor().getPosition();
  }

  @Override
  public void setScaleFactor(final double factor) {
    getSensor().setPositionConversionFactor(factor);
  }

  @Override
  public void initSendable(final SendableBuilder builder) {
    builder.addDoubleProperty("Position", this::getPosition, null);
    builder.addDoubleProperty("Raw", this::getRawValue, null);
    builder.addDoubleProperty("Factor", () -> getSensor().getPositionConversionFactor(), 
      this::setScaleFactor);
  }

  private SparkMaxAnalogSensor getSensor() {
    return controller.getAnalog(Mode.kAbsolute);
  }
}
