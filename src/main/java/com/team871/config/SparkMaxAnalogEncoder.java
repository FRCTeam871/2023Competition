package com.team871.config;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAnalogSensor;
import edu.wpi.first.util.sendable.SendableBuilder;

public class SparkMaxAnalogEncoder implements PitchEncoder {

  private final CANSparkMax CANSparkMax;
  private double zeroOffset;
  private double degreesPerVolt;

  public SparkMaxAnalogEncoder(CANSparkMax CANSparkMaxx, double zeroOffset, double degreesPerVolt) {

    this.CANSparkMax = CANSparkMaxx;
    this.zeroOffset = zeroOffset;
    this.degreesPerVolt = degreesPerVolt;
  }

  public double calculateDegrees(double rawInput) {
    return (rawInput - zeroOffset) * degreesPerVolt;
  }

  @Override
  public double getPitch() {
    return calculateDegrees(
        CANSparkMax.getAnalog(SparkMaxAnalogSensor.Mode.kAbsolute).getPosition());
  }

  @Override
  public double getRawValue() {
    return CANSparkMax.getAnalog(SparkMaxAnalogSensor.Mode.kAbsolute).getPosition();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("pitch", this::getPitch, null);
    builder.addDoubleProperty("rawValue", this::getRawValue, null);
  }

  @Override
  public void reset() {
    //
    // CANSparkMax.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle).setZeroOffset(0);
  }
}
