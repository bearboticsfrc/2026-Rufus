// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import bearlib.fms.AllianceColor;
import bearlib.fms.AllianceReadyListener;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.FuelHunt;
import frc.robot.field.AllianceFlipUtil;
import frc.robot.field.Field;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.TurretAbsolutePosition;
import frc.robot.subsystems.TurretController;
import frc.robot.util.HubTracker;

@Logged
public class RobotContainer implements AllianceReadyListener {
  private double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController joystick = new CommandXboxController(0);

  private final CommandXboxController operator = new CommandXboxController(1);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private final FuelHunt fuelHunt = new FuelHunt(drivetrain);

  public final Turret turret = new Turret();

  public final HubTracker hubTracker = new HubTracker();

  public final TurretController turretController =
      new TurretController(
          () -> drivetrain.getPose(), value -> turret.moveToAngle(value), () -> turret.getAngle());

  public final TurretAbsolutePosition TurretAbsolutePosition =
      new frc.robot.subsystems.TurretAbsolutePosition();

  /* Path follower */
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser("SimpleAuto");
    SmartDashboard.putData("Auto Mode", autoChooser);

    configureDefaultCommands();
    configureBindings();
    // Warmup PathPlanner to avoid Java pauses
    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    AllianceColor.addListener(this);
  }

  private void configureDefaultCommands() {
    turret.setDefaultCommand(turret.setAngle(() -> turret.getDefaultAngle()));
  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        -joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        -joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        -joystick.getRightX()
                            * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled()
        .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on left bumper press
    // joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    drivetrain.registerTelemetry(logger::telemeterize);
    addTurretTestBindings();
    bindPointToHubTrigger();

    joystick.rightTrigger().whileTrue(fuelHunt);
  }

  private void addTurretTestBindings() {
    operator
        .a()
        .whileTrue(
            turret.setAngle(
                () -> (Degrees.of(0)).minus(drivetrain.getPose().getRotation().getMeasure())));
    operator
        .b()
        .whileTrue(
            turret.setAngle(
                () -> (Degrees.of(90)).minus(drivetrain.getPose().getRotation().getMeasure())));
    operator
        .x()
        .whileTrue(
            turret.setAngle(
                () -> (Degrees.of(-90)).minus(drivetrain.getPose().getRotation().getMeasure())));
    operator
        .y()
        .whileTrue(
            turret.setAngle(
                () -> (Degrees.of(180)).minus(drivetrain.getPose().getRotation().getMeasure())));

    // operator.leftBumper().whileTrue(turretController.startTrackingCommand());
    operator.leftBumper().whileTrue(turret.setAngle(() -> turret.getLeftAngle()));
    operator.rightBumper().whileTrue(turret.setAngle(() -> turret.getOutpostAngle()));

    operator
        .rightTrigger()
        .whileTrue(
            turret.setAngle(
                () ->
                    Radians.of((Math.atan2(operator.getRightX(), operator.getRightY())))
                        .minus(drivetrain.getPose().getRotation().getMeasure())));
  }

  public void bindPointToHubTrigger() {
    operator
        .leftTrigger()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    driveFacingAngle
                        .withVelocityX(-joystick.getLeftY() * MaxSpeed)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                        .withHeadingPID(18, 0, .1)
                        .withTargetDirection(getAngleToHub())));
  }

  public Rotation2d getAngleToHub() {
    return AllianceFlipUtil.apply(
        (Field.getMyHub().minus(drivetrain.getPose().getTranslation())).getAngle());
  }

  public Command getAutonomousCommand() {
    /* Run the path selected from the auto chooser */
    return autoChooser.getSelected();
  }

  public void simulationPeriodic() {}

  private boolean initialPoseSet = false;

  @Override
  public void updateAlliance(Alliance alliance) {
    if (!initialPoseSet) {
      drivetrain.resetPose(
          AllianceFlipUtil.apply(((PathPlannerAuto) autoChooser.getSelected()).getStartingPose()));
      initialPoseSet = true;
    }
  }
}
