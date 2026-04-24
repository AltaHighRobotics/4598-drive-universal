package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
  private Constants() {}

  public static final class DriveConstants {
    private DriveConstants() {}

    public static final String kCanBusName = "Drivetrain";

    public static final int kFrontRightDriveMotorPort = 1;
    public static final int kBackRightDriveMotorPort = 2;
    public static final int kFrontLeftDriveMotorPort = 3;
    public static final int kBackLeftDriveMotorPort = 4;

    public static final int kFrontRightTurningMotorPort = 5;
    public static final int kBackRightTurningMotorPort = 6;
    public static final int kFrontLeftTurningMotorPort = 7;
    public static final int kBackLeftTurningMotorPort = 8;

    public static final int kFrontRightEncoderPort = 9;
    public static final int kBackRightEncoderPort = 10;
    public static final int kFrontLeftEncoderPort = 11;
    public static final int kBackLeftEncoderPort = 12;

    public static final int kPigeonIMUPort = 13;

    // Calibrate these in Phoenix Tuner after modules are mechanically aligned forward.
    public static final double kFrontLeftEncoderOffset = 0.0;
    public static final double kFrontRightEncoderOffset = 0.0;
    public static final double kBackLeftEncoderOffset = 0.0;
    public static final double kBackRightEncoderOffset = 0.0;

    public static final boolean kFrontRightDriveMotorReversed = false;
    public static final boolean kBackRightDriveMotorReversed = false;
    public static final boolean kFrontLeftDriveMotorReversed = false;
    public static final boolean kBackLeftDriveMotorReversed = false;

    public static final boolean kFrontRightTurningMotorReversed = false;
    public static final boolean kBackRightTurningMotorReversed = false;
    public static final boolean kFrontLeftTurningMotorReversed = false;
    public static final boolean kBackLeftTurningMotorReversed = false;

    // SDS MK4i L2 defaults.
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0);
    public static final double kDriveMotorGearRatio = 6.75;
    public static final double kTurningMotorGearRatio = 150.0 / 7.0;

    public static final double kTrackWidth = Units.inchesToMeters(21.5);
    public static final double kWheelBase = Units.inchesToMeters(21.5);

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
        new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
        new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
        new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0));

    public static final double kStartingPoseXMeters = 0.0;
    public static final double kStartingPoseYMeters = 0.0;
    public static final double kStartingPoseHeadingDegrees = 0.0;

    public static final String kOdometryTableName = "Odometry";
    public static final String kPoseXKey = "xMeters";
    public static final String kPoseYKey = "yMeters";
    public static final String kPoseHeadingKey = "headingDegrees";

    public static final double kMaxSpeedMetersPerSecond = 4.5;
    public static final double kMaxAngularSpeed = 2.0 * Math.PI;

    public static final double kPTurning = 10.0;
    public static final double kITurning = 0.0;
    public static final double kDTurning = 0.2;

    public static final double kDriveAccelTimeSeconds = 0.2;
    public static final double kRotAccelTimeSeconds = 0.2;
    public static final double kDriveSlewRate = 1.0 / kDriveAccelTimeSeconds;
    public static final double kRotationalSlewRate = 1.0 / kRotAccelTimeSeconds;

    // If absolute and integrated wheel angle differ by more than this, re-sync module angle.
    public static final double kAbsSyncThresholdRotations = 0.05;

    public static final class CoRConstants {
      private CoRConstants() {}

      public static final double kForwardOffsetInches = 8.0;
      public static final double kBackOffsetInches = 8.0;
      public static final double kLeftOffsetInches = 8.0;
      public static final double kRightOffsetInches = 8.0;
    }

    // Defensive wheel lock pattern after robot idles.
    public static final double kSecondsIdleForPattern = 0.0;
    public static final double kFrontLeftIdleAngleDegrees = 45.0;
    public static final double kFrontRightIdleAngleDegrees = 135.0;
    public static final double kBackLeftIdleAngleDegrees = 315.0;
    public static final double kBackRightIdleAngleDegrees = 225.0;
  }

  public static final class OIConstants {
    private OIConstants() {}

    public static final int kDriverControllerPort = 0;
    public static final int kDriverJoystickPort = 1;

    public static final double kDriveDeadband = 0.1;
    public static final double kDriveExpo = 1.8;
    public static final double kRotExpo = 1.6;

    public static final double kJoystickMaxDriveScale = 0.30;
    public static final double kJoystickTurnActivationThreshold = 0.20;
    public static final double kJoystickTurnMaxOutput = 0.50;
    public static final double kJoystickTurnSigmoidGain = 6.0;

    public static final int kJoystickXAxis = 0;
    public static final int kJoystickYAxis = 1;
    public static final int kJoystickTwistAxis = 2;
    public static final int kJoystickFullSpeedOverrideButton = 2;
  }
}
