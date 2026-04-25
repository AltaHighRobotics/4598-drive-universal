# 4598 ICARUS INNOVATED DriveBase Template
A template for 4598's drivebase, taking advantage of SDS MK4i swerve modules and kracken X60 motors. All of this code has been copied from our 25-26 Rebuilt code base, stripped down to just the drive related code and turned into a template we can use in our future years.

## Configuration
All configuration for this setup is done in a single `Constants.java` in the root of the source code. You are able to define things like CAD ids, power limits, gear ratios, PID values, motor inversions, wheelbase sizing, offset CoR amounts, and several other core part of your robot's main driving funtionality.

The current configuration has been set up for the following robot configuration:

Drive Motor: Kracken X60\
Turn Motor: Kracken X60\
Module Type: SDS MK4i (L2)\
Wheels: Both Colson and Billet work fine\
Drivebase size: 27x27in (wheels 23x23in)

## Template Use:
When you use this template, it is encouraged to follow the clean code practices that are applied here. If you look at the structure, in `src/main/java/frc/robot` all robot code is present. There are also folders (`subsystems/`, `autonomous/`) that are used to group those elements of code together, as they are mostly changed together. The `a.txt` file in `autonoumous/` can be safely deleted after you have atleast one other file there, it is only a placeholder. You should continue this layout for ease of development in the future. You should keep your `Robot.java` as minimal as possible. You really should only be ticking the periodic loops of your subsystems and reading the controller inputs with it, everything else either belongs in a subsystem or is a subsystem in itself. The `Main.java` File should never be editted. For context, this is what the layout looked like for 4598's Rebuilt (2026) season, for an example of what gets it's own subsystem:
```
.
├── autonomous (autonomous folder)
│   ├── AutoCommands.java (PathPlanner command based emulator patch)
│   ├── AutonomousBuilder.java (PathPlanner stuff)
│   └── PathPlannerEventsHandler.java (Path planner command based emulator machine handler)
├── Constants.java (Robot configurations)
├── Main.java (WPILib thing)
├── RobotConfig.java (PathPlanner configuration)
├── Robot.java (main periodic ticker)
└── subsystems (subsystems folder)
    ├── DriveSubsystem.java (drive stuff, kinematics piped to SwerveModule.java)
    ├── FeederSubsystem.java (shooter feeder stuff)
    ├── IntakeSubsystem.java (intake stuff)
    ├── ShooterSubsystem.java (shooter stuff)
    ├── SwerveModule.java (invidividual swerve module abstractions)
    └── VisionSubsystem.java (photonvision abstractions)
```

## CAN
A lookup table for the default CAN assumptions of this codebase are below. 1-13 (all items in this table) should be on their own Canivore named "Drivetrain", with all other actuators and non drivebase related components on the Roborio's CAN bus. This is done for reliability and **is not optional**.

| Name     | ID | Description            |
|----------|----|------------------------|
| FR drive | 1  | Front Right Drive     |
| BR drive | 2  | Back Right Drive      |
| FL drive | 3  | Front Left Drive      |
| BL drive | 4  | Back Left Drive       |
| FR turn  | 5  | Front Right Turn      |
| BR turn  | 6  | Back Right Turn       |
| FL turn  | 7  | Front Left Turn       |
| BL turn  | 8  | Back Left Turn        |
| FR enc   | 9  | Front Right Encoder   |
| BR enc   | 10 | Back Right Encoder    |
| FL enc   | 11 | Front Left Encoder    |
| BL enc   | 12 | Back Left Encoder     |
| GYRO     | 13 | PIGEON                |

> **NOTE: All items not in this table are to be networked via the Roborio CAN network.**

## Utilization of the current drive subsystem
The main functions that you are going to be interacting with in this subsystem are `resetOdometry`, `getPose`, `drive`, `zeroHeading`, and `getHeading`.

### resetOdometry
This is to be called on the start of your autonomous, and never called for the rest of the match under normal operation. It tells your robot that it is at 0,0 on the feild, which is important for PathPlanner to function correctly. If you need relative odometry, make a relative/delta tracker of the information from this function as best practice. Your feild position should be kept as perfect as possible. This will need to be later refined to correct position error with MegaTag with photonvision cameras, however, we didn't have the camera for the Rebuilt season this year.

This function has no parameters.

### getPose
This function returns the current pose (position on the feild) of the robot using the wheel odometry. This returns a couple items, so below is an example of how to use it where they positions are simply logged to SmartDashboard

```
    Pose2d pose = odometry.getPoseMeters();
    SmartDashboard.putNumber("Odometry X (m)", pose.getX());
    SmartDashboard.putNumber("Odometry Y (m)", pose.getY());
    SmartDashboard.putNumber("Odometry Heading (deg)", pose.getRotation().getDegrees());
```

This function has no parameters.

### drive
This is the main drive function of the robot, it is the highest level of abstraction for moving the drivebase that exists, and it handles almost everything you would need for you. This includes feild oriented driving using the Pigeon 2.0, and slew rate limiting (soft accelerations) using the values defined via `Constants.java`. There are two footprints for the function, so they will be explained below.

The first method is the most basic:
```
drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative)
```
It takes in this order, a double for lateral speed (lateral movement) known as `xSpeed`, a double for forward and backwards movement known as `ySpeed`, a double for the rotation percentage known as `rot`, and a final boolean `feildRelative` where it will drive based off of the gyro's understanding of forward (usually set to 0 when the robot's forward is forward relative to the feild, hence feild relative naming) with false making it drive with the robot's forward and true making it drive with the gyro/feild's forward.

The second method is a bit more advanced, and is largely leveraged when you are countering really aggressive defense by introducing a method of changing the swerve drive's center of rotation. 

```
drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, Translation2d centerOfRotation)
```

Everything here, omitting the Translation2D for the centerOfRotation is the same as the previous method, so they will not be described again. The new variable, `centerOfRotation` takes in a Translation2D object, which is a 2D point relative to robot center in a standard Cartesian coordinate format. The unit is **inches**. For example, with the example drivebase being 23x23 inches, sending a move command with no movement other than rotation (say 50% right if it helps to imagine) with a Translation2D of 11.5, 11.5, the robot would turn right pivoting on the front right swerve module. This is extemely useful due to how obscure this method of movement is, it can allow you to suprise your oponents by moving in a way they aren't expecting, as well as help you get around their bumber just enough to get past them in certain situations. Again, this is an advanced feature, don't waste too much time integrating this unless it's particularly useful for some reason.

### zeroHeading
This is a very simple function, it is mostly used when you need to reset the heading for your feild oriented driving due to heavy gyro drift or something similar. The button it is currently bound to is *left dpad* on the XBOX controller input (input 0). In an ideal world this is never called other than the start of the autonomous opmode, however, with games involving shooting and flywheels, constant vibration will cause some drift in the heading reading. 

This function has no parameters.

### getHeading
This is the method to get the heading of the robot, in other words, it will tell you where the robot is pointing. The unit is **degrees**. Simple, no paramaterns and you get a double of the heading returned.

This function has no parameters.

## Conlusion
This is the minimal working drive code for a standard 4598 drive base. Use this repository as a template to build something amazing, without wasting your time to redo the entire drive base and drive subsystems. 

Made with 🤖❤️ (robot love) by 4598 ICARUS INNOVATED
