// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import yams.mechanisms.swerve.utility.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods. Instead, the structure of the 
 * robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  // 1. Controller and Subsystem Setup
  private final CommandXboxController driverXbox = new CommandXboxController(0);
  private final SwerveDriveSubsystem swerve = new SwerveDriveSubsystem();
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  // 2. Properly deadbanded and inverted Input Stream
  private final SwerveInputStream driveAngularVelocity = swerve.getAngularVelocityStream(
      () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), 0.1),  // Forward/Backward (Inverted)
      () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), 0.1),  // Left/Right strafe (Inverted)
      () -> MathUtil.applyDeadband(-driverXbox.getRightX(), 0.1)  // Rotation bound to Right Stick X
  ).withAllianceRelativeControl();

  public RobotContainer()
  {
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings.
   */
  private void configureBindings()
  {
    // Continuously execute drive logic using our input stream
    swerve.setDefaultCommand(swerve.drive(driveAngularVelocity));

    // Bind your SysId characterization command to the A button (Button 1)
    //driverXbox.a().whileTrue(swerve.sysIdModule("frontleft"));

    // Bind Gyro zeroing/resetting to the Y button for quick on-field recovery
    driverXbox.y().onTrue(swerve.zeroGyro());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    return autoChooser.getSelected();
  }
}