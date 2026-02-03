// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import org.photonvision.PhotonCamera;

//import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommand;
// Import the subsystems here 
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private Command m_autonomousCommand;

  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();

  // Sensor if we need it later 
  //public static DigitalInput coralSensor = new DigitalInput(7);

private int m_rainbowFirstPixelHue=0;

  XboxController driverController = new XboxController(0);
  CommandXboxController operatorController = new CommandXboxController(1);

  AddressableLED m_led = new AddressableLED(9);
  AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(96);

  Limelight limelight = new Limelight();
  PhotonCamera camera = new PhotonCamera("EleCamera");
  DriveCommand drivetrain ;

  
  SendableChooser<Command> autoChooser = new SendableChooser<>();


Trigger intakeAlgaeIn;
  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */

  @Override
  public void robotInit() {

    configureButtonBindings();

    m_led.setLength(96);
    m_led.start();

    //Named commands for the pathplanner auto

    //NamedCommands.registerCommand("IntakeFuel", intake.startFuelIntake(SET SPEED););
    
    FollowPathCommand.warmupCommand().schedule();

    autoChooser = AutoBuilder.buildAutoChooser();

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putString("Selected Auto:", autoChooser.getSelected().getName());

    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {

    //turns each LED off when the robot is disabled
    for (int i = 0; i < 96; i++) {
      m_ledBuffer.setRGB(i, 0, 0, 0); // grb
    }
    m_led.setData(m_ledBuffer);
  }

  @Override
  public void disabledPeriodic() {
    // SmartDashboard.putNumber("FID", limelight.getFid());
    // SmartDashboard.putNumber("Intake Current", examplePD.getCurrent(2));

  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {

    m_autonomousCommand = getAutonomousCommand();

    // swerve.setLocation(0,0,0);
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove`
    // this line or comment it out.
    

    //continuously runs the DriveCommand. If some other command requires the swerve, that one will take priority
     drivetrain = new DriveCommand(swerve, driverController, operatorController, limelight, camera);
    swerve.setDefaultCommand(drivetrain);




    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //sets the LED color based on which alliance we're on
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == DriverStation.Alliance.Red) {
        for (int i = 0; i < 96; i++) {
          m_ledBuffer.setRGB(i, 0, 255, 0); // grb
        }
      } else {
        for (int i = 0; i < 96; i++) {
          m_ledBuffer.setRGB(i, 0, 0, 100); // grb
        }
      }
      // if (driverController.getYButton()){
      //   intake.setPivotAngle(10);
      // } else{
      //   intake.setPivotAngle(0);
      // }
      // if (driverController.getXButton()){
      //   intake.setFuelIntakeSpeed(1);
      // } else{
      //   intake.setFuelIntakeSpeed(0);
      // }


      //if auto aiming, set every other LED white, if aiming for algae make them cyan
      if(driverController.getLeftBumperButton()){
        for (int i = 1; i < 94; i += 2) {
          m_ledBuffer.setRGB(i, 255, 255, 0); // grb
        } 
      }
      }


      // for (int i = 1; i < 96; i += 1) {
      //   m_ledBuffer.setRGB(i, 0, 0, 0); // grb
      // } 

// sigma = sigma+1;
// if(sigma>250){
// sigma=0;
// }
//       for (int i = 1; i < 96; i += 1) {
//         m_ledBuffer.setRGB(i, sigma, 0, 0); // grb
//       } 
      // rainbow();
      m_led.setData(m_ledBuffer);
    
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {

  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {

  }

  public Command getAutonomousCommand() {

    return autoChooser.getSelected();
  }

  public void configureButtonBindings() {
    // Triggers
    // +=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=
    //  Intake 
    Trigger startIntake = operatorController.a();
    Trigger intakeUp = operatorController.povUp(); 
    Trigger intakeDown = operatorController.povDown(); 
    Trigger reverseIntake = operatorController.b();
    //shoot buttons
    Trigger shoot = operatorController.rightBumper();


    //Commands/Bindings 
    startIntake.whileTrue(intake.startFuelIntakeCmd(1));
    intakeUp.whileTrue(intake.intakePivot(0)); 
    intakeDown.whileTrue(intake.intakePivot(12));
    reverseIntake.whileTrue(intake.startFuelIntakeCmd(-.5));
    shoot.whileTrue(shooter.shootCommand());

  }
  private void rainbow() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength()-1; i = i + 1) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      // Set the value
      m_ledBuffer.setHSV(i, hue, 255, 128);

    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 2;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
  }
}
