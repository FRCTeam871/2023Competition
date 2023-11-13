package com.team871.subsystems;

import com.team871.sensor.AbsoluteEncoder;
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
  private final AbsoluteEncoder pitchEncoder;
  private boolean motorsEnabled = true;
  private final double lowClamp;
  private final double highClamp;
  private final ArmFeedforward armFeedforward;
  private double lastPosition;
  private double lastTime;
  private double minSetpoint;
  private double maxSetpoint;

  public void setMotorsEnabled(boolean motorsEnabled) {
    this.motorsEnabled = motorsEnabled;
  }

  public boolean getMotorsEnabled() {
    return motorsEnabled;
  }

  public PitchSubsystem(
      final MotorController motor,
      final AbsoluteEncoder pitchEncoder,
      double kp,
      double ki,
      double kd,
      final double lowClamp,
      final double highClamp,
      final String subsystemName,
      double kg,
      double kv,
      double minSetpoint,
      double maxSetpoint,
      double tolerance) {
    super(new PIDController(kp, ki, kd));
    this.minSetpoint = minSetpoint;
    this.maxSetpoint = maxSetpoint;
    setName(subsystemName);

    this.motor = motor;
    this.pitchEncoder = pitchEncoder;
    this.lowClamp = lowClamp;
    this.highClamp = highClamp;
    this.armFeedforward = new ArmFeedforward(0, kg, kv);
    getController().setTolerance(tolerance);

    SmartDashboard.putData(subsystemName + "-PitchPID", getController());
    SmartDashboard.putData(subsystemName + "-PitchEncoder", pitchEncoder);
    SmartDashboard.putData(subsystemName + "-DisableMotorsCommand", disableMotors());
    SmartDashboard.putData(subsystemName + "-EnableMotorsCommand", enableMotors());
  }

  public void movePitchFeedForward(final double output) {
    double currentTime = Timer.getFPGATimestamp();
    double currentPosition = pitchEncoder.getPosition();
    double velocityDegPerS = (currentPosition - lastPosition) / (currentTime - lastTime);

    // TODO: We are doing this completely wrong.  These parameters are the setpoints, not the
    // current state of the system
    double outputFeedForward =
        armFeedforward.calculate(
            Math.toRadians(pitchEncoder.getPosition()), Math.toRadians(velocityDegPerS));
    double clampedOutput =
        MathUtil.clamp(output + outputFeedForward, lowClamp * 12, highClamp * 12);
    if (motorsEnabled) {
      motor.setVoltage(clampedOutput);
    } else {
      motor.setVoltage(0);
    }

    lastTime = currentTime;
    lastPosition = currentPosition;
    SmartDashboard.putNumber(getName() + "-actualMotorOutput", clampedOutput);
    SmartDashboard.putNumber(getName() + "-rawMotorOutput", output);
    SmartDashboard.putNumber(getName() + "-velocityDegreesPerSec", velocityDegPerS);
    SmartDashboard.putNumber(getName() + "-feedForward", outputFeedForward);
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
    return pitchEncoder.getPosition();
  }

  public double getPosition() {
    return pitchEncoder.getPosition();
  }

  @Override
  public void setSetpoint(double setpoint) {
    super.setSetpoint(Math.min(maxSetpoint, Math.max(minSetpoint, setpoint)));
  }

  public Command run(String name, Runnable action) {
    final CommandBase actual = run(action);
    actual.setName(name);
    return actual;
  }
}
