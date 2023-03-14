package com.team871.subsystems;

import com.team871.config.PitchEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class PitchSubsystem extends PIDSubsystem {
  private final MotorController motor;
  private final PitchEncoder pitchEncoder;
  private boolean motorsEnabled = true;
  private final double lowClamp;
  private final double highClamp;
  private final ArmFeedforward armFeedforward;
  private double lastPosition;
  private double lastTime;

  public void setMotorsEnabled(boolean motorsEnabled) {
    this.motorsEnabled = motorsEnabled;
  }

  public boolean getMotorsEnabled() {
    return motorsEnabled;
  }

  public PitchSubsystem(
      final MotorController motor,
      final PitchEncoder pitchEncoder,
      double kp,
      double ki,
      double kd,
      final double lowClamp,
      final double highClamp,
      final String subsystemName,
      double kg,
      double kv) {
    super(new PIDController(kp, ki, kd));
    setName(subsystemName);

    this.motor = motor;
    this.pitchEncoder = pitchEncoder;
    this.lowClamp = lowClamp;
    this.highClamp = highClamp;
    this.armFeedforward = new ArmFeedforward(0, kg , kv);
    getController().setTolerance(10);
    SmartDashboard.putData(subsystemName + "-PitchPID", getController());
    SmartDashboard.putData(subsystemName + "-PitchEncoder", pitchEncoder);
    SmartDashboard.putData(subsystemName + "-DisableMotorsCommand", disableMotors());
    SmartDashboard.putData(subsystemName + "-EnableMotorsCommand", enableMotors());
  }

  public void movePitch(final double output) {
    double clampedOutput = MathUtil.clamp(output, lowClamp, highClamp);
    if (motorsEnabled) {
      motor.set(clampedOutput);
    } else {
      motor.set(0);
    }
    SmartDashboard.putNumber(getName() + "-motorOutput", clampedOutput);
    SmartDashboard.putNumber(getName() + "-rawMotorOutput", output);
  }

  public void movePitchFeedForward(final double output) {
    double currentTime = Timer.getFPGATimestamp();
    double currentPosition = pitchEncoder.getPitch();
    double velocityDegPerS = (currentPosition-lastPosition)/(currentTime-lastTime);

    // TODO: We are doing this completely wrong.  These parameters are the setpoints, not the current state of the system
    double outputFeedForward = armFeedforward.calculate(Math.toRadians(pitchEncoder.getPitch()), Math.toRadians(velocityDegPerS));
    double clampedOutput = output + outputFeedForward;
    if (motorsEnabled) {
      motor.setVoltage(clampedOutput);
    } else {
      motor.setVoltage(0);
    }
    lastTime = currentTime;
    lastPosition = currentPosition;
    SmartDashboard.putNumber(getName() + "-motorOutput", clampedOutput);
    SmartDashboard.putNumber(getName() + "-rawMotorOutput", output);
    SmartDashboard.putNumber(getName() + "velocityDegreesPerSec", velocityDegPerS);
    SmartDashboard.putNumber(getName() + "feedForward", outputFeedForward);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addBooleanProperty("motorsEnabled", this::getMotorsEnabled, this::setMotorsEnabled);
  }

  public CommandBase disableMotors() {
    return runOnce(() -> motorsEnabled = false);
  }

  public CommandBase enableMotors() {
    return runOnce(() -> motorsEnabled = true);
  }

  public boolean isAtSetpoint() {
    return getController().atSetpoint();
  }

  @Override
  protected void useOutput(double output, double setpoint) {
    movePitchFeedForward(output);
  }

  @Override
  protected double getMeasurement() {
    return pitchEncoder.getPitch();
  }

  public Command run(String name, Runnable action) {
    final CommandBase actual = run(action);
    actual.setName(name);
    return actual;
  }
}
