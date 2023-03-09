package com.team871.subsystems;

import com.team871.config.IGyro;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

public class DriveTrain extends SubsystemBase {

  private static final double BALANCE_PID_KP = 0.005;
  private static final double BALANCE_PID_KI = 0;
  private static final double BALANCE_PID_KD = 0;

  private static final double ROTATION_PID_KP = 0.02;
  private static final double ROTATION_PID_KI = 0;
  private static final double ROTATION_PID_KD = 0;

  private final MecanumDrive mecanum;
  private final IGyro gyro;
  private final PIDController balancePID;
  private final PIDController rotationPID;

  private boolean motorsEnabled = true;

  public DriveTrain(
      final MotorController frontLeftMotor,
      final MotorController frontRightMotor,
      final MotorController backLeftMotor,
      final MotorController backRightMotor,
      final IGyro gyro) {
    super();
    this.mecanum = new MecanumDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);
    this.gyro = gyro;
    this.balancePID = new PIDController(BALANCE_PID_KP, BALANCE_PID_KI, BALANCE_PID_KD);
    this.rotationPID = new PIDController(ROTATION_PID_KP, ROTATION_PID_KI, ROTATION_PID_KD);

    SmartDashboard.putData("DisableMotorsCommand", disableMotors());
    SmartDashboard.putData("EnableMotorsCommand", enableMotors());
    SmartDashboard.putData("BalanceCommand", balanceCommand());
    SmartDashboard.putData("BalancePID", balancePID);
    SmartDashboard.putData("RotationPID", rotationPID);
  }

  @Override
  public void initSendable(final SendableBuilder builder) {
    super.initSendable(builder);
    builder.addBooleanProperty("MotorStatus", this::isMotorsEnabled, null);
  }

  public boolean isMotorsEnabled() {
    return motorsEnabled;
  }

  private void driveMecanum(final double xValue, final double yValue, final double zValue) {
    SmartDashboard.putNumber("mecanumX", xValue);
    SmartDashboard.putNumber("mecanumY", yValue);
    SmartDashboard.putNumber("mecanumZ", zValue);
    if (motorsEnabled) {
      mecanum.driveCartesian(xValue, yValue, zValue * .35);
    } else {
      mecanum.driveCartesian(0, 0, 0);
    }
  }

  public CommandBase driveMechanumCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotationSupplier) {
    final CommandBase defaultCommand =
        run(() -> driveMecanum(
                exponentialDrive(xSupplier.getAsDouble()),
                exponentialDrive(ySupplier.getAsDouble()),
                exponentialDrive(rotationSupplier.getAsDouble())));

    defaultCommand.setName("DriveMechanumCommand");
    return defaultCommand;
  }

  public double exponentialDrive(double controllerOutput) {
    double contollerOutputA = 10;
    double controllerOutputB = 0.015;
    double controllerOutputC = (1-controllerOutputB)/(contollerOutputA-1);
    double wrappedControllerOutput =
        controllerOutputC* Math.pow(contollerOutputA, Math.abs(controllerOutput)) + controllerOutputB * Math.abs(controllerOutput) - controllerOutputC;
    if (controllerOutput >= 0) {
      return wrappedControllerOutput;
    } else {
      return -wrappedControllerOutput;
    }
  }

  public CommandBase balanceCommand() {
    final CommandBase command =
        new PIDCommand(
                balancePID,
                gyro::getPitch,
                0,
                output -> {
                  final double rotationPIDOutput = rotationPID.calculate(gyro.getYaw());
                  SmartDashboard.putNumber("pitchPIDOutput", output);
                  SmartDashboard.putNumber("yawPIDOutput", rotationPIDOutput);
                  // positive pitch should be forward and negative pitch should be backwards
                  driveMecanum(-output, 0, rotationPIDOutput);
                },
                this)
            .beforeStarting(
                () -> {
                  rotationPID.reset();
                  rotationPID.setSetpoint(0);
                });

    command.setName("BalanceCommand");
    return command;
  }

  public CommandBase disableMotors() {
    return runOnce(() -> motorsEnabled = false);
  }

  public CommandBase enableMotors() {
    return runOnce(() -> motorsEnabled = true);
  }

  public CommandBase driveForwardCommand(final double speed) {
    return run(() -> driveMecanum(speed, 0, rotationPID.calculate(gyro.getYaw())));
  }

  public CommandBase driveDurationCommand(final double speed, final double duration) {
    return driveForwardCommand(speed).withTimeout(duration).andThen(driveForwardCommand(0));
  }

  public Command stopCommand() {
    return run(() -> driveMecanum(0, 0, 0));
  }

  public CommandBase rotateCommand(final double degrees) {
    return new PIDCommand(
        rotationPID, gyro::getYaw, degrees, output -> driveMecanum(0, 0, output), this);
  }
}
