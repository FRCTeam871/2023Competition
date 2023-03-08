package com.team871.subsystems;

import com.team871.config.PitchEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
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
      final String subsystemName) {
    this.wristMotor = motor;
    this.pitchPID = new PIDController(kp, ki, kd);
    this.pitchEncoder = pitchEncoder;
    this.subsystemName = subsystemName;
    this.lowClamp = lowClamp;
    this.highClamp = highClamp;
    SmartDashboard.putData(subsystemName + "-PitchPID", pitchPID);
    SmartDashboard.putData(subsystemName + "-PitchEncoder", pitchEncoder);
    SmartDashboard.putData(subsystemName + "-DisableMotorsCommand", disableMotors());
    SmartDashboard.putData(subsystemName + "-EnableMotorsCommand", enableMotors());
  }

  public void movePitch(final double output) {
    double clampedOutput = MathUtil.clamp(output, lowClamp, highClamp);
    // double clampedOutput = output;
    if (motorsEnabled) {
      // wristMotor.set(output);
      wristMotor.set(clampedOutput);
    } else {
      motor.set(0);
    }
    SmartDashboard.putNumber(subsystemName + "-motorOutput", clampedOutput);
  }

  public boolean isMotorsEnabled() {
    return motorsEnabled;
  }

  //  public void setdefaultCommand(Supplier<Double> wristPitch) {
  //    setDefaultCommand(run(()-> wristPitchPIDCommand()));
  ////    setDefaultCommand(run(() -> moveWristPitch(wristPitch.get())));
  //  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty(
        "ThetaSetpoint", this::getPositionThetaSetpointTest, this::setPositionThetaSetpointTest);
    builder.addBooleanProperty("motorsEnabled", this::getMotorsEnabled, this::setMotorsEnabled);
    builder.addDoubleProperty("directMotorOutput", this::getDirectMotorOutputTest, this::setDirectMotorOutputTest);
  }

  public CommandBase pitchPIDCommand(DoubleSupplier setpointSupplier) {
    final CommandBase command =
        new PIDCommand(pitchPID, pitchEncoder::getPitch, setpointSupplier, this::movePitch, this);

    command.setName("PitchCommand");
    return command;
  }
  // jules waz here =)
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

  public CommandBase setOutputTest() {
    return run(()-> this.motor.set(directMotorOutputTest));
  }
}
