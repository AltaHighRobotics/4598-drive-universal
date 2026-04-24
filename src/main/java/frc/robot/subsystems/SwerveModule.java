package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;

public class SwerveModule {
  private final TalonFX driveMotor;
  private final TalonFX turningMotor;
  private final CANcoder absoluteEncoder;
  private final double absoluteOffset;

  private final PositionVoltage turningPositionControl = new PositionVoltage(0.0);
  private final VelocityVoltage driveVelocityControl = new VelocityVoltage(0.0);

  private double lastKP = DriveConstants.kPTurning;
  private double lastKI = DriveConstants.kITurning;
  private double lastKD = DriveConstants.kDTurning;

  private static GenericEntry kPEntry;
  private static GenericEntry kIEntry;
  private static GenericEntry kDEntry;
  private static boolean shuffleboardInitialized = false;

  private final String moduleID;
  private boolean hasSynced = false;

  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int absoluteEncoderChannel,
      double absoluteEncoderOffset,
      boolean driveMotorReversed,
      boolean turningMotorReversed) {
    moduleID = "Module_" + driveMotorChannel;

    driveMotor = new TalonFX(driveMotorChannel, DriveConstants.kCanBusName);
    turningMotor = new TalonFX(turningMotorChannel, DriveConstants.kCanBusName);
    absoluteEncoder = new CANcoder(absoluteEncoderChannel, DriveConstants.kCanBusName);
    absoluteOffset = absoluteEncoderOffset;

    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    driveConfig.MotorOutput.Inverted = driveMotorReversed
        ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.Slot0.kP = 0.1;
    driveConfig.Slot0.kI = 0.0;
    driveConfig.Slot0.kD = 0.0;
    driveConfig.Slot0.kV = 0.12;
    driveConfig.CurrentLimits.SupplyCurrentLimit = 40;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveMotor.getConfigurator().apply(driveConfig);

    TalonFXConfiguration turningConfig = new TalonFXConfiguration();
    turningConfig.MotorOutput.Inverted = turningMotorReversed
        ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;
    turningConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turningConfig.Slot0.kP = DriveConstants.kPTurning;
    turningConfig.Slot0.kI = DriveConstants.kITurning;
    turningConfig.Slot0.kD = DriveConstants.kDTurning;
    turningConfig.ClosedLoopGeneral.ContinuousWrap = false;
    turningConfig.CurrentLimits.SupplyCurrentLimit = 40;
    turningConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    turningMotor.getConfigurator().apply(turningConfig);

    if (!shuffleboardInitialized) {
      ShuffleboardTab tuningTab = Shuffleboard.getTab("Swerve Tuning");
      kPEntry = tuningTab.add("Turning kP", DriveConstants.kPTurning).getEntry();
      kIEntry = tuningTab.add("Turning kI", DriveConstants.kITurning).getEntry();
      kDEntry = tuningTab.add("Turning kD", DriveConstants.kDTurning).getEntry();
      shuffleboardInitialized = true;
    }

    // The module is explicitly synced by DriveSubsystem alignment routine on startup.
    driveMotor.setPosition(0);
  }

  public boolean alignToZero() {
    syncTurnEncoderToAbsolute(true);
    hasSynced = true;
    return true;
  }

  public void updateTurnEncoderFromAbsolute() {
    syncTurnEncoderToAbsolute(false);
  }

  private void syncTurnEncoderToAbsolute(boolean force) {
    double absRot = absoluteEncoder.getAbsolutePosition().getValueAsDouble() * -1;
    double corrected = normalizeAbsoluteRotation(absRot - absoluteOffset);

    double currentMotorRotations = turningMotor.getPosition().getValueAsDouble();
    double currentWheelRotations = currentMotorRotations / DriveConstants.kTurningMotorGearRatio;

    double remainder = currentWheelRotations % 1.0;
    if (remainder < 0.0) {
      remainder += 1.0;
    }

    double error = corrected - remainder;
    if (error > 0.5) {
      error -= 1.0;
    } else if (error < -0.5) {
      error += 1.0;
    }

    if (force || !hasSynced || Math.abs(error) > DriveConstants.kAbsSyncThresholdRotations) {
      double newWheelRotations = currentWheelRotations + error;
      turningMotor.setPosition(newWheelRotations * DriveConstants.kTurningMotorGearRatio);
      hasSynced = true;
    }
  }

  private double normalizeAbsoluteRotation(double rotation) {
    rotation %= 1.0;
    if (rotation < 0.0) {
      rotation += 1.0;
    }
    return rotation;
  }

  public void updatePIDFromDashboard() {
    double kP = kPEntry.getDouble(DriveConstants.kPTurning);
    double kI = kIEntry.getDouble(DriveConstants.kITurning);
    double kD = kDEntry.getDouble(DriveConstants.kDTurning);

    if (kP != lastKP || kI != lastKI || kD != lastKD) {
      TalonFXConfiguration config = new TalonFXConfiguration();
      config.Slot0.kP = kP;
      config.Slot0.kI = kI;
      config.Slot0.kD = kD;
      turningMotor.getConfigurator().apply(config);
      lastKP = kP;
      lastKI = kI;
      lastKD = kD;
    }
  }

  public SwerveModulePosition getPosition() {
    double drivePositionMeters =
        (driveMotor.getPosition().getValueAsDouble() / DriveConstants.kDriveMotorGearRatio)
            * (DriveConstants.kWheelDiameterMeters * Math.PI);
    return new SwerveModulePosition(drivePositionMeters, new Rotation2d(getTurningPosition()));
  }

  public SwerveModuleState getState() {
    double driveVelocityMetersPerSecond =
        (driveMotor.getVelocity().getValueAsDouble() / DriveConstants.kDriveMotorGearRatio)
            * (DriveConstants.kWheelDiameterMeters * Math.PI);
    return new SwerveModuleState(driveVelocityMetersPerSecond, new Rotation2d(getTurningPosition()));
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    if (!hasSynced) {
      stop();
      return;
    }

    if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }

    Rotation2d currentAngle = new Rotation2d(getTurningPosition());
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, currentAngle);

    double driveVelocityRPS =
        (state.speedMetersPerSecond / (DriveConstants.kWheelDiameterMeters * Math.PI))
            * DriveConstants.kDriveMotorGearRatio;

    double currentMotorRotations = turningMotor.getPosition().getValueAsDouble();
    double currentWheelRotations = currentMotorRotations / DriveConstants.kTurningMotorGearRatio;

    double targetWheelRotations = state.angle.getRotations();

    double remainder = currentWheelRotations % 1.0;
    if (remainder < 0.0) {
      remainder += 1.0;
    }

    double delta = targetWheelRotations - remainder;
    if (delta > 0.5) {
      delta -= 1.0;
    } else if (delta < -0.5) {
      delta += 1.0;
    }

    double finalTargetWheelRotations = currentWheelRotations + delta;
    double turningPositionRotations =
        finalTargetWheelRotations * DriveConstants.kTurningMotorGearRatio;

    driveMotor.setControl(driveVelocityControl.withVelocity(driveVelocityRPS));
    turningMotor.setControl(turningPositionControl.withPosition(turningPositionRotations));

    SmartDashboard.putNumber(moduleID + "/Current Angle", currentAngle.getDegrees());
    SmartDashboard.putNumber(moduleID + "/Target Angle", state.angle.getDegrees());
    SmartDashboard.putNumber(moduleID + "/Delta Deg", Math.toDegrees(delta * 2.0 * Math.PI));
    SmartDashboard.putNumber(moduleID + "/Speed mps", state.speedMetersPerSecond);
  }

  public void setIdleAngle(double angleDegrees) {
    if (!hasSynced) {
      return;
    }

    double targetRotations = angleDegrees / 360.0;

    double currentMotorRotations = turningMotor.getPosition().getValueAsDouble();
    double currentWheelRotations = currentMotorRotations / DriveConstants.kTurningMotorGearRatio;

    double remainder = currentWheelRotations % 1.0;
    if (remainder < 0.0) {
      remainder += 1.0;
    }

    double delta = targetRotations - remainder;
    if (delta > 0.5) {
      delta -= 1.0;
    } else if (delta < -0.5) {
      delta += 1.0;
    }

    double finalTargetWheelRotations = currentWheelRotations + delta;
    double turningPositionRotations =
        finalTargetWheelRotations * DriveConstants.kTurningMotorGearRatio;

    turningMotor.setControl(turningPositionControl.withPosition(turningPositionRotations));
    SmartDashboard.putNumber(moduleID + "/Idle Target Deg", angleDegrees);
  }

  private double getTurningPosition() {
    double motorRotations = turningMotor.getPosition().getValueAsDouble();
    double wheelRotations = motorRotations / DriveConstants.kTurningMotorGearRatio;
    return wheelRotations * 2.0 * Math.PI;
  }

  public void resetEncoders() {
    driveMotor.setPosition(0);
    turningMotor.setPosition(0);
    hasSynced = false;
  }

  public void stop() {
    driveMotor.set(0.0);
  }

  public void setDriveMotorBrakeMode(boolean brake) {
    MotorOutputConfigs motorOutput = new MotorOutputConfigs();
    motorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    driveMotor.getConfigurator().apply(motorOutput);
  }

  public double getCurrentAngleDegrees() {
    double degrees = Math.toDegrees(getTurningPosition());
    degrees %= 360.0;
    if (degrees < 0.0) {
      degrees += 360.0;
    }
    return degrees;
  }

  public double getAbsoluteEncoder() {
    return absoluteEncoder.getAbsolutePosition().getValueAsDouble();
  }
}
