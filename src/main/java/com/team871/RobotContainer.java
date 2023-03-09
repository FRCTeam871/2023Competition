// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team871;

import com.team871.config.*;
import com.team871.dashboard.DriveTrainExtensions;
import com.team871.simulation.SimulationGyro;
import com.team871.simulation.SimulationPitchEncoder;
import com.team871.subsystems.ArmExtension;
import com.team871.subsystems.Claw;
import com.team871.subsystems.DriveTrain;
import com.team871.subsystems.Intake;
import com.team871.subsystems.PitchSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
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

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    config = new RobotConfig();
    controlConfig = new XboxHotasControlConfig();
    gyro = RobotBase.isReal() ? new Gyro() : new SimulationGyro();

    drivetrain =
        new DriveTrain(
            config.getFrontLeftMotor(),
            config.getFrontRightMotor(),
            config.getRearLeftMotor(),
            config.getRearRightMotor(),
            gyro);

    final PitchEncoder shoulderPitchEncoder =
        RobotBase.isSimulation() ? new SimulationPitchEncoder() : config.getShoulderPitchEncoder();

    // -90 is fully up, 0 is parallel to the ground, 90 is fully down. Down is negative motor output
    shoulder =
        new PitchSubsystem(
            config.getShoulderMotor(),
            shoulderPitchEncoder,
            0.032,
            0,
            0,
            config.getShoulderLowClampValue(),
            config.getShoulderHighClampValue(),
            "Shoulder");

    final PitchEncoder wristPitchEncoder =
        RobotBase.isSimulation() ? new SimulationPitchEncoder() : config.getWristPitchEncoder();

    /**
     * 90 is fully up, 0 is parallel to the ground, -90 is fully down. Down is positive motor output
     */
    wrist =
        new PitchSubsystem(config.getWristMotor(), wristPitchEncoder, 0.048, 0, 0, -1, 1, "Wrist");
    claw = new Claw(config.getClawMotor());
    intake = new Intake(config.getLeftIntakeMotor(), config.getRightIntakeMotor());
    armExtension = new ArmExtension(config.getArmExtensionMotor(), config.getExtensionEncoder());

    SmartDashboard.putData("DriveTrain", drivetrain);
    SmartDashboard.putData("Shoulder", shoulder);
    SmartDashboard.putData("Wrist", wrist);
    SmartDashboard.putData("Claw", claw);
    SmartDashboard.putData("ArmExtension", armExtension);
    SmartDashboard.putData("Gyro", gyro);
    SmartDashboard.putData("DriveTrainTest", new DriveTrainExtensions(drivetrain));

    // Configure the trigger bindings
    configureBindings();

    // Suppress "Joystick Button 2 on port 0 not available, check if controller is plugged in"
    // flooding in console
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    System.out.println("configure bindings");

    configureDrivetrainControllerBindings();
    configureShoulderBindings();
    configureClawBindings();
    configureWristBindings();
    configureIntakeBindings();
    configureArmExtensionBindings();
  }

  private void configureClawBindings() {
    claw.setDefaultCommand(controlConfig::getClawAxisValue);
  }

  public void configureCompositeCommands(){
    controlConfig.getFoldOutTrigger()
        .toggleOnTrue(
            shoulder.pitchPIDCommand("Bottom",
      () -> {
        final double targetPosition = config.getBottomShoulderSetpoint();
        final double offsetValue = controlConfig.getShoulderAxisValue() * config.getMaxOffsetShoulderValue();
        return targetPosition + offsetValue;
      }).andThen(armExtension.extensionPIDCommand(
        "bottom", config::getBottomExtensionSetpoint)));
  }

  private void configureWristBindings() {
    wrist.setDefaultCommand(
        wrist.pitchPIDCommand("FollowShoulder",
            () -> {
              final double targetPosition = config.getShoulderPitchEncoder().getPitch();
              final double offsetValue = controlConfig.getWristAxisValue() * config.getMaxOffsetWristValue();
              return (targetPosition * -1) + offsetValue;
            }));
  }

  private void configureShoulderBindings() {
    shoulder.setDefaultCommand(
        shoulder.pitchPIDCommand("PickUp",
            () -> (controlConfig.getShoulderAxisValue() * 45) + config.getRestOnFrameSetpoint()));

    controlConfig.getHighNodeTrigger().toggleOnTrue(shoulder.pitchPIDCommand("HighNode",
      () -> {
        final double targetPosition = config.getTopShoulderSetpoint();
        final double offsetValue = controlConfig.getShoulderAxisValue() * config.getMaxOffsetShoulderValue();
        return targetPosition + offsetValue;
      }));

    controlConfig.getMiddleNodeTrigger()
        .toggleOnTrue(
            shoulder.pitchPIDCommand("MiddleNode",
      () -> {
        final double targetPosition = config.getMiddleShoulderSetpoint();
        final double offsetValue = controlConfig.getShoulderAxisValue() * config.getMaxOffsetShoulderValue();
        return targetPosition + offsetValue;
      }));

    controlConfig.getBottomNodeTrigger()
        .toggleOnTrue(
            shoulder.pitchPIDCommand("Bottom",
      () -> {
        final double targetPosition = config.getBottomShoulderSetpoint();
        final double offsetValue = controlConfig.getShoulderAxisValue() * config.getMaxOffsetShoulderValue();
        return targetPosition + offsetValue;
      }));
  }

  private void configureArmExtensionBindings() {
    armExtension.setDefaultCommand(armExtension.extensionPIDCommand("joystickSetpoint", ()-> {
      double joystickSetpoint = (-controlConfig.getExtensionAxisValue() + 1)*(19.0/2);
      return joystickSetpoint;
    }
    ));
    controlConfig.getHighNodeTrigger()
        .toggleOnTrue(
            armExtension.extensionPIDCommand(
                "TopNode", config::getTopExtensionSetpoint));
    controlConfig.getMiddleNodeTrigger()
        .toggleOnTrue(
            armExtension.extensionPIDCommand(
                "MiddleNode", config::getMiddleExtensionSetpoint));
    controlConfig.getBottomNodeTrigger()
        .toggleOnTrue(
            armExtension.extensionPIDCommand(
                "BottomNode", config::getBottomExtensionSetpoint));
  }

  private void configureIntakeBindings() {
    controlConfig.getIntakeTrigger().whileTrue(intake.run(intake::pullIn));
    controlConfig.getExhaustTrigger().whileTrue(intake.run(intake::pullOut));
  }

  private void configureDrivetrainControllerBindings() {
    drivetrain.setDefaultCommand(
            drivetrain.driveMechanumCommand(
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
    // An example command will be run in autonomous
    //        return Autos.exampleAuto(exampleSubsystem);
    return null;
  }
}
