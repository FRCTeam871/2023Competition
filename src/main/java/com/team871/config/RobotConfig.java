package com.team871.config;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.PIDController;
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

  private final double maxShoulderOffsetValue = 20;
  private static final double shoulderZero = 1.44;
  private static final double shoulderNegative90Value = 2.2478;

  private final double maxWristOffsetValue = 45;
  private static final double wristZeroOffset = -635;

  private static final double topShoulderSetpoint = -12.7;
  /** formerly 16.2 */
  private static final double middleShoulderSetpoint = 13.2;
  /** formerly 62 */
  private static final double bottomShoulderSetpoint = 58;

  private static final double topExtensionSetpoint = 18;
  private static final double middleExtensionSetpoint = 0;
  private static final double bottomExtensionSetpoint = 15;
  private static final double pickupExtensionSetpoint = 15;

  private static final double restOnFrameSetpoint = 62;

  private static final double foldOutShoulderSetpoint = 55;
  private static final double foldOutExtensionSetpoint = 14;

  private static final double foldInShoulderSetpoint = 90;
  private static final double foldInExtensionSetpoint = 0;

  private static final double lowClamp = -1;
  private static final double highClamp = .1;

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

    clawMotor = new WPI_TalonSRX(9);
    clawMotor.setNeutralMode(NeutralMode.Brake);

    armExtensionMotor = new WPI_TalonSRX(8);
    armExtensionMotor.setNeutralMode(NeutralMode.Brake);
    armExtensionMotor.setInverted(true);
    armExtensionMotor.configForwardLimitSwitchSource(
        LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    armExtensionMotor.configReverseLimitSwitchSource(
        LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    armExtensionMotor.configClearPositionOnLimitR(true, 0);
    armExtensionMotor.configPeakCurrentLimit(20);
    armExtensionMotor.configPeakCurrentDuration(250);

    balancePID = new PIDController(0.03, 0.0, 0.0001);

    gyro = new Gyro();

    extensionEncoder = new SRXDistanceEncoder(armExtensionMotor, 0.00007005);
    // up 90 degrees is 380 down 90 degrees is 900, original value for degrees per tick was -.3529
    final double wristDegreesPerTick = 180.0d / (380.0d - 900.0d);
    wristPitchEncoder =
        new SRXAnalogEncoderTalonSRX(wristMotor, wristZeroOffset, wristDegreesPerTick);
    // down 90 is 1.5 and striaght out (0 degrees) is .68
    final double shoulderDegreesPerVolt = 90 / (shoulderNegative90Value - shoulderZero);
    shoulderPitchEncoder =
        new SparkMaxAnalogEncoder(shoulderMotor, shoulderZero, shoulderDegreesPerVolt);
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
  public double getMaxOffsetWristValue() {
    return maxWristOffsetValue;
  }

  @Override
  public double getMaxOffsetShoulderValue() {
    return maxShoulderOffsetValue;
  }

  @Override
  public double getTopShoulderSetpoint() {
    return topShoulderSetpoint;
  }

  @Override
  public double getMiddleShoulderSetpoint() {
    return middleShoulderSetpoint;
  }

  @Override
  public double getBottomShoulderSetpoint() {
    return bottomShoulderSetpoint;
  }

  @Override
  public double getTopExtensionSetpoint() {
    return topExtensionSetpoint;
  }

  @Override
  public double getMiddleExtensionSetpoint() {
    return middleExtensionSetpoint;
  }

  @Override
  public double getBottomExtensionSetpoint() {
    return bottomExtensionSetpoint;
  }

  @Override
  public double getShoulderLowClampValue() {
    return lowClamp;
  }

  @Override
  public double getShoulderHighClampValue() {
    return highClamp;
  }

  @Override
  public double getRestOnFrameSetpoint() {
    return restOnFrameSetpoint;
  }

  @Override
  public double getFoldInShoulderSetpoint() {
    return foldInShoulderSetpoint;
  }

  @Override
  public double getFoldInExtensionSetpoint() {
    return foldInExtensionSetpoint;
  }

  @Override
  public double getFoldOutShoulderSetpoint() {
    return foldOutShoulderSetpoint;
  }

  @Override
  public double getFoldOutExtensionSetpoint() {
    return foldOutExtensionSetpoint;
  }

  public BooleanSupplier getIsExtensionRetracted() {
    return () -> armExtensionMotor.isRevLimitSwitchClosed() == 1;
  }

  @Override
  public double getPickupExtensionSetpoint() {
    return pickupExtensionSetpoint;
  }
}
