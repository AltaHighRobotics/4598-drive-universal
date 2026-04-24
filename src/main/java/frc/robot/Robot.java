package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

public class Robot extends TimedRobot {
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();

  private final XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);
  private final Joystick driverJoystick = new Joystick(OIConstants.kDriverJoystickPort);

  private boolean lastUpDPad = false;
  private boolean lastLeftDPad = false;

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    // Leave enabled if you later wire in a path/auto controller.
    driveSubsystem.setAutoDriveEnabled(true);
  }

  @Override
  public void autonomousPeriodic() {
    // Intentionally empty in this drive-only starter.
  }

  @Override
  public void teleopInit() {
    // Prevent stale auto commands from writing chassis speeds in teleop.
    driveSubsystem.setAutoDriveEnabled(false);
  }

  @Override
  public void teleopPeriodic() {
    int pov = driverController.getPOV();
    boolean upDPad = pov == 0;
    boolean leftDPad = pov == 270;

    if (leftDPad && !lastLeftDPad) {
      driveSubsystem.zeroHeading();
      System.out.println("LEFT D-PAD PRESSED - Gyro heading reset to 0");
    }
    lastLeftDPad = leftDPad;

    if (upDPad && !lastUpDPad) {
      driveSubsystem.toggleDriveMotorBrakeMode();
      System.out.println("UP D-PAD PRESSED - Toggling brake mode");
    }
    lastUpDPad = upDPad;

    double xSpeedXbox = -driverController.getLeftY();
    double ySpeedXbox = driverController.getLeftX();
    double rotXbox = driverController.getRightX();

    double xSpeedJoy = -driverJoystick.getRawAxis(OIConstants.kJoystickYAxis);
    double ySpeedJoy = driverJoystick.getRawAxis(OIConstants.kJoystickXAxis);
    double rotJoy = driverJoystick.getRawAxis(OIConstants.kJoystickTwistAxis);

    xSpeedXbox = applyDeadband(xSpeedXbox);
    ySpeedXbox = applyDeadband(ySpeedXbox);
    rotXbox = applyDeadband(rotXbox);
    xSpeedJoy = applyDeadband(xSpeedJoy);
    ySpeedJoy = applyDeadband(ySpeedJoy);
    rotJoy = applyDeadband(rotJoy);

    boolean fullSpeedOverrideHeld =
        driverJoystick.getRawButton(OIConstants.kJoystickFullSpeedOverrideButton);
    double joystickDriveScale = fullSpeedOverrideHeld ? 1.0 : OIConstants.kJoystickMaxDriveScale;
    xSpeedJoy *= joystickDriveScale;
    ySpeedJoy *= joystickDriveScale;
    rotJoy = shapeJoystickTurn(rotJoy);

    boolean xboxActive = (xSpeedXbox != 0.0) || (ySpeedXbox != 0.0) || (rotXbox != 0.0);

    double xSpeed = xboxActive ? xSpeedXbox : xSpeedJoy;
    double ySpeed = xboxActive ? ySpeedXbox : ySpeedJoy;
    double rot = xboxActive ? rotXbox : rotJoy;

    xSpeed = applyDeadband(xSpeed);
    ySpeed = applyDeadband(ySpeed);
    rot = applyDeadband(rot);

    if (xboxActive) {
      xSpeed = applyExpo(xSpeed, OIConstants.kDriveExpo);
      ySpeed = applyExpo(ySpeed, OIConstants.kDriveExpo);
      rot = applyExpo(rot, OIConstants.kRotExpo);
    }

    SmartDashboard.putBoolean("Drive/Joystick Full Speed Override", fullSpeedOverrideHeld);

    if (xSpeed == 0.0 && ySpeed == 0.0 && rot == 0.0) {
      driveSubsystem.stopModules();
      return;
    }

    Translation2d centerOfRotation = getCenterOfRotationFromPOV();
    driveSubsystem.drive(xSpeed, ySpeed, rot, true, centerOfRotation);
  }

  @Override
  public void disabledInit() {
    driveSubsystem.setAutoDriveEnabled(false);
    driveSubsystem.stopModules();
    driveSubsystem.homeModules();
  }

  private double applyDeadband(double value) {
    return Math.abs(value) < OIConstants.kDriveDeadband ? 0.0 : value;
  }

  private double applyExpo(double value, double expo) {
    return Math.copySign(Math.pow(Math.abs(value), expo), value);
  }

  private double shapeJoystickTurn(double turnInput) {
    double magnitude = Math.abs(turnInput);
    if (magnitude < OIConstants.kJoystickTurnActivationThreshold) {
      return 0.0;
    }

    double normalizedMagnitude =
        (magnitude - OIConstants.kJoystickTurnActivationThreshold)
            / (1.0 - OIConstants.kJoystickTurnActivationThreshold);
    double sigmoidMagnitude =
        normalizedSigmoid01(normalizedMagnitude, OIConstants.kJoystickTurnSigmoidGain);
    double outputMagnitude = sigmoidMagnitude * OIConstants.kJoystickTurnMaxOutput;
    return Math.copySign(outputMagnitude, turnInput);
  }

  private double normalizedSigmoid01(double input01, double gain) {
    double clampedInput = MathUtil.clamp(input01, 0.0, 1.0);
    double centeredInput = clampedInput - 0.5;
    double sigmoid = 1.0 / (1.0 + Math.exp(-gain * centeredInput));

    double low = 1.0 / (1.0 + Math.exp(gain * 0.5));
    double high = 1.0 / (1.0 + Math.exp(-gain * 0.5));
    double span = high - low;
    if (Math.abs(span) < 1e-9) {
      return clampedInput;
    }
    return MathUtil.clamp((sigmoid - low) / span, 0.0, 1.0);
  }

  private Translation2d getCenterOfRotationFromPOV() {
    double xInches = 0.0;
    double yInches = 0.0;
    int pov = driverJoystick.getPOV();

    if (pov == 315) {
      xInches = DriveConstants.CoRConstants.kForwardOffsetInches;
      yInches = DriveConstants.CoRConstants.kLeftOffsetInches;
    } else if (pov == 45) {
      xInches = DriveConstants.CoRConstants.kForwardOffsetInches;
      yInches = -DriveConstants.CoRConstants.kRightOffsetInches;
    } else if (pov == 225) {
      xInches = -DriveConstants.CoRConstants.kBackOffsetInches;
      yInches = DriveConstants.CoRConstants.kLeftOffsetInches;
    } else if (pov == 135) {
      xInches = -DriveConstants.CoRConstants.kBackOffsetInches;
      yInches = -DriveConstants.CoRConstants.kRightOffsetInches;
    }

    return new Translation2d(Units.inchesToMeters(xInches), Units.inchesToMeters(yInches));
  }
}
