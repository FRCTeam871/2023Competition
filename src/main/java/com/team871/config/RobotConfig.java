package com.team871.config;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.team871.simulation.SimulationDistanceEncoder;
import com.team871.simulation.SimulationPitchEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import java.util.function.BooleanSupplier;

public class RobotConfig implements IRobot {
  private final CANSparkMax frontLeft;
  private final CANSparkMax frontRight;
  private final CANSparkMax rearLeft;
  private final CANSparkMax rearRight;
  private final CANSparkMax shoulderMotor;
  private final CANSparkMax leftIntakeMotor;
  private final CANSparkMax rightIntakeMotor;
  private final WPI_TalonSRX wristMotor;
  private final WPI_TalonSRX clawMotor;
  private final WPI_TalonSRX armExtensionMotor;

  private final PIDController balancePID;

  private final IGyro gyro;
  private final DistanceEncoder extensionEncoder;
  private final PitchEncoder wristPitchEncoder;
  private final PitchEncoder shoulderPitchEncoder;

  // region Shoulder constants
  private static final double SHOULDER_TRIM_RANGE_DEG = 20;
  private static final double SHOULDER_ENCODER_ZERO_VALUE = 1.44;
  private static final double SHOULDER_ENCODER_NEG_90_VALUE = 2.2478;
  private static final double SHOULDER_TOP_SETPOINT_DEG = -12.7;
  private static final double SHOULDER_MIDDLE_SETPOINT_DEG = 13.2;
  private static final double SHOULDER_BOTTOM_SETPOINT_DEG = 55;
  private static final double SHOULDER_FOLDED_SETPOINT_DEG = 90;

  // Note that negative values rotate the arm UP and positive are DOWN
  private static final double SHOULDER_MINIMUM_OUTPUT_PERCENT = -1;
  private static final double SHOULDER_MAXIMUM_OUTPUT_PERCENT = .1;

  // endregion

  // region Wrist constants
  private static final double WRIST_TRIM_RANGE_DEG = 45;
  private static final double WRIST_ENCODER_ZERO_VALUE = -635;

  // endregion

  // region Extension constants
  private static final double EXTENSION_TOP_SETPOINT_INCHES = 18.5;
  private static final double EXTENSION_MIDDLE_SETPOINT_INCHES = 0;
  private static final double EXTENSION_BOTTOM_SETPOINT_INCHES = 15;
  private static final double EXTENSION_FOLDED_SETPOINT_INCHES = 0;

  private static final double EXTENSION_MAXIMUM_TRIM_OFFSET_INCHES = 6;
  // endregion Extension Constants


  public RobotConfig() {
    /* sets front left motor to CanSparkMax motor controller with device id 1 */
    frontLeft = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
    frontLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);
    frontLeft.setInverted(false);

    /* sets front right motor to CanSparkMax motor controller with device id 2 */
    frontRight = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
    frontRight.setIdleMode(CANSparkMax.IdleMode.kBrake);
    frontRight.setInverted(true);

    /* sets rear left motor to CanSparkMax motor controller with device id 3 */
    rearLeft = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
    rearLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rearLeft.setInverted(false);

    /* sets rear right motor to CanSparkMax motor controller with device id 4 */
    rearRight = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
    rearRight.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rearRight.setInverted(true);

    shoulderMotor = new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless);
    shoulderMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    shoulderMotor.setInverted(true);

    /** TODO set limit switches */
    leftIntakeMotor = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);
    leftIntakeMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    leftIntakeMotor.setInverted(true);

    rightIntakeMotor = new CANSparkMax(7, CANSparkMaxLowLevel.MotorType.kBrushless);
    rightIntakeMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightIntakeMotor.setInverted(false);

    wristMotor = new WPI_TalonSRX(10);
    wristMotor.setNeutralMode(NeutralMode.Brake);
    wristMotor.setInverted(true);
    wristMotor.configPeakCurrentLimit(15);
    wristMotor.configPeakCurrentDuration(250);
    wristMotor.configContinuousCurrentLimit(5);
    wristMotor.enableCurrentLimit(true);

    clawMotor = new WPI_TalonSRX(9);
    clawMotor.setNeutralMode(NeutralMode.Brake);
    clawMotor.configContinuousCurrentLimit(8);
    clawMotor.configPeakCurrentLimit(0);
    clawMotor.enableCurrentLimit(true);

    armExtensionMotor = new WPI_TalonSRX(8);
    armExtensionMotor.setNeutralMode(NeutralMode.Brake);
    armExtensionMotor.setInverted(true);
    armExtensionMotor.configForwardLimitSwitchSource(
        LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    armExtensionMotor.configReverseLimitSwitchSource(
        LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

    armExtensionMotor.configClearPositionOnLimitR(true, 0);
    armExtensionMotor.configPeakCurrentLimit(0);
    armExtensionMotor.configContinuousCurrentLimit(20);
    armExtensionMotor.enableCurrentLimit(true);

    balancePID = new PIDController(0.03, 0.0, 0.0001);

    gyro = new Gyro();

    extensionEncoder = RobotBase.isSimulation() ? new SimulationDistanceEncoder() : new SRXDistanceEncoder(armExtensionMotor, 0.00007005);

    // up 90 degrees is 380 down 90 degrees is 900, original value for degrees per tick was -.3529
    final double wristDegreesPerTick = 180.0d / (380.0d - 900.0d);
    wristPitchEncoder = RobotBase.isSimulation() ? new SimulationPitchEncoder() : new SRXAnalogEncoderTalonSRX(wristMotor, WRIST_ENCODER_ZERO_VALUE, wristDegreesPerTick);
    // down 90 is 1.5 and striaght out (0 degrees) is .68
    final double shoulderDegreesPerVolt = 90 / (SHOULDER_ENCODER_NEG_90_VALUE - SHOULDER_ENCODER_ZERO_VALUE);
    shoulderPitchEncoder = RobotBase.isSimulation() ? new SimulationPitchEncoder() : new SparkMaxAnalogEncoder(shoulderMotor, SHOULDER_ENCODER_ZERO_VALUE, shoulderDegreesPerVolt);
  }

  @Override
  public MotorController getFrontLeftMotor() {
    return frontLeft;
  }

  @Override
  public MotorController getRearLeftMotor() {
    return rearLeft;
  }

  @Override
  public MotorController getFrontRightMotor() {
    return frontRight;
  }

  @Override
  public MotorController getRearRightMotor() {
    return rearRight;
  }

  @Override
  public IGyro gyro() {
    return gyro;
  }

  @Override
  public PIDController getBalancePID() {
    return balancePID;
  }

  @Override
  public DistanceEncoder getExtensionEncoder() {
    return extensionEncoder;
  }

  @Override
  public PitchEncoder getWristPitchEncoder() {
    return wristPitchEncoder;
  }

  @Override
  public PitchEncoder getShoulderPitchEncoder() {
    return shoulderPitchEncoder;
  }

  @Override
  public MotorController getShoulderMotor() {
    return shoulderMotor;
  }

  @Override
  public MotorController getArmExtensionMotor() {
    return armExtensionMotor;
  }

  @Override
  public MotorController getWristMotor() {
    return wristMotor;
  }

  @Override
  public MotorController getClawMotor() {
    return clawMotor;
  }

  @Override
  public MotorController getLeftIntakeMotor() {
    return leftIntakeMotor;
  }

  @Override
  public MotorController getRightIntakeMotor() {
    return rightIntakeMotor;
  }

  @Override
  public double getMaxWristTrimOffset() {
    return WRIST_TRIM_RANGE_DEG;
  }

  @Override
  public double getMaxShoulderTrimOffset() {
    return SHOULDER_TRIM_RANGE_DEG;
  }

  @Override
  public double getMaxExtensionTrimOffset() {
    return EXTENSION_MAXIMUM_TRIM_OFFSET_INCHES;
  }

  @Override
  public double getTopShoulderSetpoint() {
    return SHOULDER_TOP_SETPOINT_DEG;
  }

  @Override
  public double getMiddleShoulderSetpoint() {
    return SHOULDER_MIDDLE_SETPOINT_DEG;
  }

  @Override
  public double getBottomShoulderSetpoint() {
    return SHOULDER_BOTTOM_SETPOINT_DEG;
  }

  @Override
  public double getTopExtensionSetpoint() {
    return EXTENSION_TOP_SETPOINT_INCHES;
  }

  @Override
  public double getMiddleExtensionSetpoint() {
    return EXTENSION_MIDDLE_SETPOINT_INCHES;
  }

  @Override
  public double getBottomExtensionSetpoint() {
    return EXTENSION_BOTTOM_SETPOINT_INCHES;
  }

  @Override
  public double getShoulderLowClampValue() {
    return SHOULDER_MINIMUM_OUTPUT_PERCENT;
  }

  @Override
  public double getShoulderHighClampValue() {
    return SHOULDER_MAXIMUM_OUTPUT_PERCENT;
  }

  @Override
  public double getFoldInShoulderSetpoint() {
    return SHOULDER_FOLDED_SETPOINT_DEG;
  }

  @Override
  public double getFoldInExtensionSetpoint() {
    return EXTENSION_FOLDED_SETPOINT_INCHES;
  }

  public BooleanSupplier getIsExtensionRetracted() {
    return () -> armExtensionMotor.isRevLimitSwitchClosed() == 1;
  }
}
