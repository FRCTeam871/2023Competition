package com.team871.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import com.team871.sensor.IGyro;

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
  private NetworkTableEntry limelightTable;

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
    // limelightTable = NetworkTableInstance.getDefault().getEntry(getName())

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

  public void driveMecanum(final double xValue, final double yValue, final double zValue) {
    SmartDashboard.putNumber("mecanumX", xValue);
    SmartDashboard.putNumber("mecanumY", yValue);
    SmartDashboard.putNumber("mecanumZ", zValue);
    if (motorsEnabled) {
      mecanum.driveCartesian(xValue, yValue, zValue * .35);
    } else {
      mecanum.driveCartesian(0, 0, 0);
    }
  }

  public CommandBase driveMechanumCommand(
      DoubleSupplier shoulderPositionDeg,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier rotationSupplier) {

    double slope = .013;

    final CommandBase defaultCommand =
        run(
            () -> {
              double multiplier = 1;
              if (shoulderPositionDeg.getAsDouble() < 38.5) {
                multiplier = slope * (shoulderPositionDeg.getAsDouble()) + .545;
              }

              driveMecanum(
                  exponentialDrive(xSupplier.getAsDouble() * multiplier),
                  exponentialDrive(ySupplier.getAsDouble() * multiplier),
                  exponentialDrive(rotationSupplier.getAsDouble() * multiplier));
            });

    defaultCommand.setName("DriveMechanumCommand");
    return defaultCommand;
  }

  public double exponentialDrive(double controllerOutput) {
    double contollerOutputA = 15;
    double controllerOutputB = 0.015;
    double controllerOutputC = (1 - controllerOutputB) / (contollerOutputA - 1);
    double wrappedControllerOutput =
        controllerOutputC * Math.pow(contollerOutputA, Math.abs(controllerOutput))
            + controllerOutputB * Math.abs(controllerOutput)
            - controllerOutputC;
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

  public CommandBase leaveZone(final BooleanSupplier isAt45Degrees) {
    return run(() -> driveMecanum(0, .2, 0)).until(isAt45Degrees);
  }

  public Boolean isRobotAtBalanceSetpoint(DoubleSupplier gyroPitch) {
    if (gyroPitch.getAsDouble() >= 15.0d) {
      return true;
    } else {
      return false;
    }
  }

  public void autonMecanum(double xValue, double yValue, double zValue) {
    mecanum.driveCartesian(xValue, yValue, zValue);
    SmartDashboard.putNumber(getName() + "autonX", xValue);
    SmartDashboard.putNumber(getName() + "autonY", yValue);
    SmartDashboard.putNumber(getName() + "autonZ", zValue);
  }
}
