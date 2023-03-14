package com.team871.subsystems;

import com.team871.config.PitchEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class PitchSubsystem extends SubsystemBase {
  private final MotorController motor;
  private final PIDController pitchPID;
  private final PitchEncoder pitchEncoder;
  private double positionThetaSetpointTest;
  private String subsystemName;
  private boolean motorsEnabled = true;
  private final double lowClamp;
  private final double highClamp;
  private ArmFeedforward armFeedforward;
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
    this.motor = motor;
    this.pitchPID = new PIDController(kp, ki, kd);
    pitchPID.setTolerance(10);
    this.pitchEncoder = pitchEncoder;
    this.subsystemName = subsystemName;
    this.lowClamp = lowClamp;
    this.highClamp = highClamp;
    armFeedforward = new ArmFeedforward(0, kg, kv);
    SmartDashboard.putData(subsystemName + "-PitchPID", pitchPID);
    SmartDashboard.putData(subsystemName + "-PitchEncoder", pitchEncoder);
    SmartDashboard.putData(subsystemName + "-DisableMotorsCommand", disableMotors());
    SmartDashboard.putData(subsystemName + "-EnableMotorsCommand", enableMotors());
  }

  public void movePitch(final double output) {
    double clampedOutput = MathUtil.clamp(output, lowClamp, highClamp);
    // double clampedOutput = output;
    if (motorsEnabled) {
      // motor.set(output);
      motor.set(clampedOutput);
    } else {
      motor.set(0);
    }
    SmartDashboard.putNumber(subsystemName + "-motorOutput", clampedOutput);
    SmartDashboard.putNumber(subsystemName + "-rawMotorOutput", output);
  }

  public void movePitchFeedForward(final double output) {
    double currentTime = Timer.getFPGATimestamp();
    double currentPosition = pitchEncoder.getPitch();
    double velocityDegPerS = (currentPosition - lastPosition) / (currentTime - lastTime);
    double outputFeedForward =
        armFeedforward.calculate(
            Math.toRadians(pitchEncoder.getPitch()), Math.toRadians(velocityDegPerS));
    // double clampedOutput = MathUtil.clamp(outputFeedForward, lowClamp*12, highClamp*12);
    double clampedOutput = output + outputFeedForward;
    if (motorsEnabled) {
      // motor.set(output);
      motor.setVoltage(clampedOutput);
    } else {
      motor.setVoltage(0);
    }
    lastTime = currentTime;
    lastPosition = currentPosition;
    SmartDashboard.putNumber(subsystemName + "-motorOutput", clampedOutput);
    SmartDashboard.putNumber(subsystemName + "-rawMotorOutput", output);
    SmartDashboard.putNumber(subsystemName + "velocityDegreesPerSec", velocityDegPerS);
    SmartDashboard.putNumber(subsystemName + "feedForward", outputFeedForward);
  }

  public boolean isMotorsEnabled() {
    return motorsEnabled;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty(
        "ThetaSetpoint", this::getPositionThetaSetpointTest, this::setPositionThetaSetpointTest);
    builder.addBooleanProperty("motorsEnabled", this::getMotorsEnabled, this::setMotorsEnabled);
  }

  public PIDCommand pitchPIDCommand(String name, DoubleSupplier setpointSupplier) {
    final PIDCommand command =
        new PIDCommand(pitchPID, pitchEncoder::getPitch, setpointSupplier, this::movePitch, this);

    command.setName(name);
    return command;
  }

  public PIDCommand pitchPIDFeedForwardCommand(String name, DoubleSupplier setpointSupplier) {
    final PIDCommand command =
        new PIDCommand(
            pitchPID, pitchEncoder::getPitch, setpointSupplier, this::movePitchFeedForward, this);

    command.setName(name);
    return command;
  }

  public double getPositionThetaSetpointTest() {
    return positionThetaSetpointTest;
  }

  public void setPositionThetaSetpointTest(double positionThetaSetpointTest) {
    this.positionThetaSetpointTest = positionThetaSetpointTest;
  }

  public CommandBase disableMotors() {
    return runOnce(() -> motorsEnabled = false);
  }

  public CommandBase enableMotors() {
    return runOnce(() -> motorsEnabled = true);
  }

  public boolean isAtSetpoint() {
    return pitchPID.atSetpoint();
  }
}
