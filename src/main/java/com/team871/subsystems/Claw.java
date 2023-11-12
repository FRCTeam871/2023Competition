package com.team871.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;

public class Claw extends SubsystemBase {

  private final MotorController pinchMotor;

  public Claw(final MotorController pinchMotor) {
    this.pinchMotor = pinchMotor;
  }

  public void setPinch(final double output) {
    pinchMotor.set(output);
    SmartDashboard.putNumber(getName() + "pinchMotorOutput", output);
  }

  public void setDefaultCommand(Supplier<Double> pinchSupplier) {
    setDefaultCommand(run(() -> setPinch(pinchSupplier.get())));
  }
}
