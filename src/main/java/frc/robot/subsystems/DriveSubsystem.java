package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  private final SwerveModule frontLeft = new SwerveModule(
      DriveConstants.kFrontLeftDriveMotorPort,
      DriveConstants.kFrontLeftTurningMotorPort,
      DriveConstants.kFrontLeftEncoderPort,
      DriveConstants.kFrontLeftEncoderOffset,
      DriveConstants.kFrontLeftDriveMotorReversed,
      DriveConstants.kFrontLeftTurningMotorReversed);

  private final SwerveModule frontRight = new SwerveModule(
      DriveConstants.kFrontRightDriveMotorPort,
      DriveConstants.kFrontRightTurningMotorPort,
      DriveConstants.kFrontRightEncoderPort,
      DriveConstants.kFrontRightEncoderOffset,
      DriveConstants.kFrontRightDriveMotorReversed,
      DriveConstants.kFrontRightTurningMotorReversed);

  private final SwerveModule backLeft = new SwerveModule(
      DriveConstants.kBackLeftDriveMotorPort,
      DriveConstants.kBackLeftTurningMotorPort,
      DriveConstants.kBackLeftEncoderPort,
      DriveConstants.kBackLeftEncoderOffset,
      DriveConstants.kBackLeftDriveMotorReversed,
      DriveConstants.kBackLeftTurningMotorReversed);

  private final SwerveModule backRight = new SwerveModule(
      DriveConstants.kBackRightDriveMotorPort,
      DriveConstants.kBackRightTurningMotorPort,
      DriveConstants.kBackRightEncoderPort,
      DriveConstants.kBackRightEncoderOffset,
      DriveConstants.kBackRightDriveMotorReversed,
      DriveConstants.kBackRightTurningMotorReversed);

  private final Pigeon2 gyro = new Pigeon2(DriveConstants.kPigeonIMUPort, DriveConstants.kCanBusName);

  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      getRotation2d(),
      getModulePositions());

  private final NetworkTable odometryTable =
      NetworkTableInstance.getDefault().getTable(DriveConstants.kOdometryTableName);
  private final NetworkTableEntry poseXEntry = odometryTable.getEntry(DriveConstants.kPoseXKey);
  private final NetworkTableEntry poseYEntry = odometryTable.getEntry(DriveConstants.kPoseYKey);
  private final NetworkTableEntry poseHeadingEntry = odometryTable.getEntry(DriveConstants.kPoseHeadingKey);

  private final SlewRateLimiter xLimiter = new SlewRateLimiter(DriveConstants.kDriveSlewRate);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(DriveConstants.kDriveSlewRate);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);

  private boolean driveMotorsBrakeMode = true;
  private boolean autoDriveEnabled = true;

  private final Timer idleTimer = new Timer();
  private boolean isIdle = false;
  private boolean inIdlePattern = false;

  private boolean alignmentComplete = false;

  public DriveSubsystem() {
    gyro.reset();
    idleTimer.start();

    resetOdometry(new Pose2d(
        DriveConstants.kStartingPoseXMeters,
        DriveConstants.kStartingPoseYMeters,
        Rotation2d.fromDegrees(DriveConstants.kStartingPoseHeadingDegrees)));
  }

  @Override
  public void periodic() {
    if (!alignmentComplete) {
      runAlignmentRoutine();
      return;
    }

    frontLeft.updateTurnEncoderFromAbsolute();
    frontRight.updateTurnEncoderFromAbsolute();
    backLeft.updateTurnEncoderFromAbsolute();
    backRight.updateTurnEncoderFromAbsolute();

    odometry.update(getRotation2d(), getModulePositions());
    publishOdometry();

    Pose2d pose = odometry.getPoseMeters();
    SmartDashboard.putNumber("Odometry X (m)", pose.getX());
    SmartDashboard.putNumber("Odometry Y (m)", pose.getY());
    SmartDashboard.putNumber("Odometry Heading (deg)", pose.getRotation().getDegrees());

    frontLeft.updatePIDFromDashboard();
    frontRight.updatePIDFromDashboard();
    backLeft.updatePIDFromDashboard();
    backRight.updatePIDFromDashboard();

    SmartDashboard.putNumber("FL Encoder (abs)", frontLeft.getAbsoluteEncoder());
    SmartDashboard.putNumber("FR Encoder (abs)", frontRight.getAbsoluteEncoder());
    SmartDashboard.putNumber("BL Encoder (abs)", backLeft.getAbsoluteEncoder());
    SmartDashboard.putNumber("BR Encoder (abs)", backRight.getAbsoluteEncoder());

    SmartDashboard.putNumber("Gyro Heading", getHeading());
    SmartDashboard.putBoolean("Alignment Complete", alignmentComplete);
    SmartDashboard.putBoolean("Drive Motors Brake Mode", driveMotorsBrakeMode);
    SmartDashboard.putBoolean("In Idle Pattern", inIdlePattern);
    SmartDashboard.putNumber("Idle Timer", idleTimer.get());
    SmartDashboard.putBoolean("Drive/Auto Drive Enabled", autoDriveEnabled);
  }

  private void publishOdometry() {
    Pose2d pose = odometry.getPoseMeters();
    poseXEntry.setDouble(pose.getX());
    poseYEntry.setDouble(pose.getY());
    poseHeadingEntry.setDouble(pose.getRotation().getDegrees());
  }

  private void runAlignmentRoutine() {
    boolean fl = frontLeft.alignToZero();
    boolean fr = frontRight.alignToZero();
    boolean bl = backLeft.alignToZero();
    boolean br = backRight.alignToZero();

    if (fl && fr && bl && br) {
      alignmentComplete = true;
      System.out.println("===================================");
      System.out.println("ALL MODULES ALIGNED TO ZERO!");
      System.out.println("Robot ready for operation");
      System.out.println("===================================");
    }
  }

  public boolean isAlignmentComplete() {
    return alignmentComplete;
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    drive(xSpeed, ySpeed, rot, fieldRelative, new Translation2d());
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, Translation2d centerOfRotation) {
    if (!alignmentComplete) {
      return;
    }

    if (xSpeed != 0.0 || ySpeed != 0.0 || rot != 0.0) {
      isIdle = false;
      inIdlePattern = false;
      idleTimer.reset();
    }

    xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kMaxSpeedMetersPerSecond;
    ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kMaxSpeedMetersPerSecond;
    rot = rotLimiter.calculate(rot) * DriveConstants.kMaxAngularSpeed;

    ChassisSpeeds chassisSpeeds = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getRotation2d())
        : new ChassisSpeeds(xSpeed, ySpeed, rot);

    SwerveModuleState[] moduleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds, centerOfRotation);

    setModuleStates(moduleStates);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
        frontLeft.getState(),
        frontRight.getState(),
        backLeft.getState(),
        backRight.getState());
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    if (!autoDriveEnabled) {
      return;
    }
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    setModuleStates(moduleStates);
  }

  public void setAutoDriveEnabled(boolean enabled) {
    autoDriveEnabled = enabled;
    SmartDashboard.putBoolean("Drive/Auto Drive Enabled", autoDriveEnabled);
  }

  public void resetEncoders() {
    frontLeft.resetEncoders();
    frontRight.resetEncoders();
    backLeft.resetEncoders();
    backRight.resetEncoders();
  }

  public void zeroHeading() {
    gyro.reset();
  }

  public double getHeading() {
    return getRotation2d().getDegrees();
  }

  public double getTurnRate() {
    return gyro.getAngularVelocityZWorld().getValueAsDouble();
  }

  private Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(-gyro.getYaw().getValueAsDouble());
  }

  private SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()
    };
  }

  public void stopModules() {
    if (!alignmentComplete) {
      return;
    }

    if (!isIdle) {
      isIdle = true;
      idleTimer.reset();
    }

    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();

    if (driveMotorsBrakeMode
        && !inIdlePattern
        && idleTimer.get() >= DriveConstants.kSecondsIdleForPattern) {
      enterIdlePattern();
    }
  }

  private void enterIdlePattern() {
    inIdlePattern = true;

    frontLeft.setIdleAngle(DriveConstants.kFrontLeftIdleAngleDegrees);
    frontRight.setIdleAngle(DriveConstants.kFrontRightIdleAngleDegrees);
    backLeft.setIdleAngle(DriveConstants.kBackLeftIdleAngleDegrees);
    backRight.setIdleAngle(DriveConstants.kBackRightIdleAngleDegrees);

    System.out.println("Entering idle pattern - wheels locked in defensive position");
  }

  public void homeModules() {
    if (!alignmentComplete) {
      return;
    }

    frontLeft.setIdleAngle(0.0);
    frontRight.setIdleAngle(0.0);
    backLeft.setIdleAngle(0.0);
    backRight.setIdleAngle(0.0);

    System.out.println("Homing all turn motors to 0 degrees");
  }

  public void toggleDriveMotorBrakeMode() {
    driveMotorsBrakeMode = !driveMotorsBrakeMode;
    frontLeft.setDriveMotorBrakeMode(driveMotorsBrakeMode);
    frontRight.setDriveMotorBrakeMode(driveMotorsBrakeMode);
    backLeft.setDriveMotorBrakeMode(driveMotorsBrakeMode);
    backRight.setDriveMotorBrakeMode(driveMotorsBrakeMode);

    if (!driveMotorsBrakeMode) {
      inIdlePattern = false;
      idleTimer.reset();
    }

    System.out.println("Drive motors set to: " + (driveMotorsBrakeMode ? "BRAKE" : "COAST"));
  }

  public boolean areWheelsHomed(double toleranceDegrees) {
    double flAngle = normalizeAngle(Math.abs(frontLeft.getCurrentAngleDegrees()));
    double frAngle = normalizeAngle(Math.abs(frontRight.getCurrentAngleDegrees()));
    double blAngle = normalizeAngle(Math.abs(backLeft.getCurrentAngleDegrees()));
    double brAngle = normalizeAngle(Math.abs(backRight.getCurrentAngleDegrees()));

    boolean flHomed = flAngle <= toleranceDegrees || flAngle >= (360.0 - toleranceDegrees);
    boolean frHomed = frAngle <= toleranceDegrees || frAngle >= (360.0 - toleranceDegrees);
    boolean blHomed = blAngle <= toleranceDegrees || blAngle >= (360.0 - toleranceDegrees);
    boolean brHomed = brAngle <= toleranceDegrees || brAngle >= (360.0 - toleranceDegrees);

    return flHomed && frHomed && blHomed && brHomed;
  }

  private double normalizeAngle(double angle) {
    angle %= 360.0;
    if (angle < 0.0) {
      angle += 360.0;
    }
    return angle;
  }

  public SwerveDriveKinematics getKinematics() {
    return DriveConstants.kDriveKinematics;
  }

  public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
        frontLeft.getState(),
        frontRight.getState(),
        backLeft.getState(),
        backRight.getState());
  }
}
