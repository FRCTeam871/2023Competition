// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team871;

import com.team871.config.*;
import com.team871.dashboard.DriveTrainExtensions;
import com.team871.simulation.SimulationGyro;
import com.team871.subsystems.ArmExtension;
import com.team871.subsystems.Claw;
import com.team871.subsystems.DriveTrain;
import com.team871.subsystems.Intake;
import com.team871.subsystems.PitchSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final IControlConfig controlConfig;

  private final DriveTrain drivetrain;
  private final PitchSubsystem shoulder;
  private final ArmExtension armExtension;
  private final PitchSubsystem wrist;
  private final Claw claw;
  private final Intake intake;
  private final IRobot config;
  private final IGyro gyro;
  private Command homeExtensionCommand;
  private Command foldInCommand;
  private Command bottomCommand;
  private Command middleCommand;
  private Command topCommand;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    config = new RobotConfig();
//    controlConfig = new SimulationDualKeyboardControl();
    controlConfig = new XboxHotasSystemsController();
    //controlConfig = new HotasOnlyControlConfig();
    gyro = RobotBase.isReal() ? new Gyro() : new SimulationGyro();

    drivetrain = new DriveTrain(
        config.getFrontLeftMotor(),
        config.getFrontRightMotor(),
        config.getRearLeftMotor(),
        config.getRearRightMotor(),
        gyro);

    final PitchEncoder shoulderPitchEncoder = config.getShoulderPitchEncoder();

    // -90 is fully up, 0 is parallel to the ground, 90 is fully down. Down is
    // negative motor output
    shoulder = new PitchSubsystem(config.getShoulderMotor(),
            shoulderPitchEncoder,
            0.11,
            0,
            0,
            config.getShoulderLowClampValue(),
            config.getShoulderHighClampValue(),
            "Shoulder",
            -1.5,
            1,
            -15,
            90);

    final PitchEncoder wristPitchEncoder = config.getWristPitchEncoder();

    /*
     * 90 is fully up, 0 is parallel to the ground, -90 is fully down. Down is
     * positive motor output
     */
    wrist = new PitchSubsystem(config.getWristMotor(),
            wristPitchEncoder,
            0.7,
            0,
            0,
            -1,
            1,
            "Wrist",
            0,
            0,
            -90,
            90);
    claw = new Claw(config.getClawMotor());
    intake = new Intake(config.getLeftIntakeMotor(), config.getRightIntakeMotor());

    final DistanceEncoder extensionEncoder = config.getExtensionEncoder();
    armExtension = new ArmExtension(config.getArmExtensionMotor(), extensionEncoder);

    SmartDashboard.putData("DriveTrain", drivetrain);
    SmartDashboard.putData("Shoulder", shoulder);
    SmartDashboard.putData("Wrist", wrist);
    SmartDashboard.putData("Claw", claw);
    SmartDashboard.putData("ArmExtension", armExtension);
    SmartDashboard.putData("Gyro", gyro);
    SmartDashboard.putData("DriveTrainTest", new DriveTrainExtensions(drivetrain));

    // Configure the trigger bindings
    configureBindings();

    wrist.enable();
    shoulder.enable();
    armExtension.enable();

    // Suppress "Joystick Button 2 on port 0 not available, check if controller is
    // plugged in"
    // flooding in console
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    configureClawBindings();
    configureWristBindings();
    configureIntakeBindings();
    configureDrivetrainBindings();
    configureExtensionBindings();

    defineCommands();
    bindCommands();
  }

  private void configureClawBindings() {
    claw.setDefaultCommand(controlConfig::getClawAxisValue);
  }

  /**
   * Define the commands as instances so that we can refer to them when we need to.  Commands in the Command Based
   * architecture are not ephemeral objects, and should be retained and reused.
   */
  private void defineCommands() {
    // Fold the arm in.  Note that we don't let the operator trim the extension here,  there's no need to
    // and it's unsafe.
    foldInCommand = Commands.run(() -> {
      armExtension.setSetpoint(config.getFoldInExtensionSetpoint());

      // If the extension is safe, we can move the shoulder all the way in
      // otherwise, go as low as we can until we get there
      if (armExtension.isAtSetpoint()) {
        shoulder.setSetpoint(config.getFoldInShoulderSetpoint());
      } else {
        shoulder.setSetpoint(config.getBottomShoulderSetpoint());
      }
    }, armExtension, shoulder)
    .until(() -> armExtension.isAtSetpoint() && shoulder.isAtSetpoint())
    .andThen(applyTrimCommand(config.getFoldInShoulderSetpoint(),
            () -> controlConfig.getShoulderAxisValue() * config.getMaxShoulderTrimOffset(),
            shoulder::setSetpoint));
    foldInCommand.setName("FoldIn");

    bottomCommand = Commands.run(() -> {
      shoulder.setSetpoint(config.getBottomShoulderSetpoint());

      // If the shoulder is poked out, it's safe to extend out
      // Otherwise pull the extension all the way in because that's safe
      if (shoulder.isAtSetpoint()) {
        armExtension.setSetpoint(config.getBottomExtensionSetpoint());
      } else {
        armExtension.setSetpoint(1);
      }
    }, armExtension, shoulder)
    .until(() -> armExtension.isAtSetpoint() && shoulder.isAtSetpoint())
    .andThen(applyTrimCommand(config.getBottomShoulderSetpoint(),
            () -> controlConfig.getShoulderAxisValue() * config.getMaxShoulderTrimOffset(),
            shoulder::setSetpoint))
    .alongWith(applyTrimCommand(config.getBottomExtensionSetpoint(),
            () -> controlConfig.getExtensionAxisValue() * config.getMaxExtensionTrimOffset(),
            armExtension::setSetpoint));
    bottomCommand.setName("Bottom");

    middleCommand = Commands.run(() -> {
      shoulder.setSetpoint(config.getMiddleExtensionSetpoint());

      // If the shoulder is poked out, it's safe to extend out
      // Otherwise pull the extension all the way in because that's safe
      if (shoulder.getPosition() < config.getBottomExtensionSetpoint()) {
        armExtension.setSetpoint(config.getMiddleExtensionSetpoint());
      } else {
        armExtension.setSetpoint(1);
      }
    }, armExtension, shoulder)
    .until(() -> armExtension.isAtSetpoint() && shoulder.isAtSetpoint())
    .andThen(applyTrimCommand(config.getMiddleShoulderSetpoint(),
            () -> controlConfig.getShoulderAxisValue() * config.getMaxShoulderTrimOffset(),
            shoulder::setSetpoint))
    .alongWith(applyTrimCommand(config.getMiddleExtensionSetpoint(),
            () ->  controlConfig.getExtensionAxisValue() * config.getMaxExtensionTrimOffset(),
            armExtension::setSetpoint));
    middleCommand.setName("Middle");

    topCommand = Commands.run(() -> {
      shoulder.setSetpoint(config.getTopShoulderSetpoint());

      // If the shoulder is poked out, it's safe to extend out
      // Otherwise pull the extension all the way in because that'syttuuihgg safe
      if (shoulder.getPosition() < config.getBottomShoulderSetpoint()) {
        armExtension.setSetpoint(config.getTopExtensionSetpoint());
      } else {
        armExtension.setSetpoint(1);
      }
    }, armExtension, shoulder)
    .until(() -> armExtension.isAtSetpoint() && shoulder.isAtSetpoint())
    .andThen(applyTrimCommand(config.getTopShoulderSetpoint(),
            () -> controlConfig.getShoulderAxisValue() * config.getMaxShoulderTrimOffset(),
            shoulder::setSetpoint))
    .alongWith(applyTrimCommand(config.getTopExtensionSetpoint(),
            () ->  controlConfig.getExtensionAxisValue() * config.getMaxExtensionTrimOffset(),
            armExtension::setSetpoint));
    topCommand.setName("Top");

    homeExtensionCommand = armExtension.homeExtensionCommand(config.getIsExtensionRetracted())
            .andThen(() -> armExtension.setSetpoint(1));
    homeExtensionCommand.setName("Home");
  }

  /**
   * Bind all the commands configured above to triggers.
   */
  private void bindCommands() {
    controlConfig.getFoldInTrigger().onTrue(foldInCommand);
    controlConfig.getBottomNodeTrigger().onTrue(bottomCommand);
    controlConfig.getMiddleNodeTrigger().onTrue(middleCommand);
    controlConfig.getHighNodeTrigger().onTrue(topCommand);
    controlConfig.getHomeExtensionTrigger().onTrue(homeExtensionCommand);
  }

  /**
   * Run a command to apply trim to a subsystem.  Reset the setpoint to the desired before the command is
   * canceled to prevent unexpected exciting and surprising behavior and generally preventing
   * Rapid Unscheduled Disassembly
   */
  private Command applyTrimCommand(final double actualSetpoint,
                                   DoubleSupplier trimSupplier,
                                   DoubleConsumer setpointConsumer) {
    return Commands.runEnd(
            () -> setpointConsumer.accept(actualSetpoint + trimSupplier.getAsDouble()),
            () -> setpointConsumer.accept(actualSetpoint));
  }

  private void configureExtensionBindings() {
    controlConfig.getManualControl().onTrue(armExtension.run("ManualControl",
      () -> {
        double setPoint = controlConfig.getExtensionAxisValue() * 19.0d;

        // Don't let an operator smash the extension into the frame.
        if(shoulder.getPosition() < config.getBottomShoulderSetpoint() ||
           shoulder.getSetpoint() < config.getBottomShoulderSetpoint()) {
          setPoint = 1;
        }

        armExtension.setSetpoint(setPoint);
      }));
  }

  private void configureWristBindings() {
    wrist.setDefaultCommand(
        wrist.run("FollowShoulder",
            () -> {
              final double targetPosition = config.getShoulderPitchEncoder().getPitch();
              final double offsetValue = controlConfig.getWristAxisValue() * config.getMaxWristTrimOffset();
              double setpoint = (targetPosition * -1) + offsetValue;

              if(shoulder.getPosition() > config.getBottomShoulderSetpoint() + 10) {
                setpoint = Math.min(-90, setpoint);
              }

              wrist.setSetpoint(setpoint);
            }));
  }

  private void configureIntakeBindings() {
    controlConfig.getIntakeTrigger().whileTrue(intake.run(intake::pullIn));
    controlConfig.getExhaustTrigger().whileTrue(intake.run(intake::pullOut));
  }

  private void configureDrivetrainBindings() {
    drivetrain.setDefaultCommand(
        drivetrain.driveMechanumCommand(shoulder::getPosition,
            controlConfig::getDriveXAxisValue,
            controlConfig::getDriveYAxisValue,
            controlConfig::getDriveRotationAxisValue));
    controlConfig.getBalanceTrigger().whileTrue(drivetrain.balanceCommand());
    controlConfig.getResetGyroTrigger().whileTrue(gyro.resetGyroCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return armExtension.homeExtensionCommand(config.getIsExtensionRetracted());
  }

  // We should always home our extension first.
  public Command getTeleopInitCommand() {
    return Commands.parallel(Commands.runOnce(() -> shoulder.setSetpoint(config.getBottomShoulderSetpoint())), 
    Commands.runOnce(() -> wrist.setSetpoint(-90))).andThen(homeExtensionCommand);
  }
}
