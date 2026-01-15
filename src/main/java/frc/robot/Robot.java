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
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;
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
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final IntakePivotSubsystem intakePivotSubsystem = new IntakePivotSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
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
  public static DigitalInput coralSensor = new DigitalInput(7);

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
    NamedCommands.registerCommand("Outtake", intake.runIntakesAuto(0.2));
    NamedCommands.registerCommand("Intake", intake.runIntakesAuto(-0.3));
    NamedCommands.registerCommand("Stop Intake", intake.runIntakes(0));

    NamedCommands.registerCommand("AlignNearestTag", swerve.alignCommand());

    NamedCommands.registerCommand("Elevator Down", elevator.elevatorDown().alongWith(intakePivotSubsystem.intakePivot(0)));
    NamedCommands.registerCommand("Elevator L1", elevator.elevatorUp(1).alongWith(intakePivotSubsystem.intakePivot(0)));
    NamedCommands.registerCommand("ElevatorL2", elevator.elevatorUp(2).alongWith(intakePivotSubsystem.intakePivot(5.5)));
    NamedCommands.registerCommand("ElevatorL3", elevator.elevatorUp(3).alongWith(intakePivotSubsystem.intakePivot(5.5)));
    NamedCommands.registerCommand("ElevatorL4", elevator.elevatorUp(4).alongWith(intakePivotSubsystem.intakePivot(6.5)));
    NamedCommands.registerCommand("AlgaeL2", elevator.elevatorUp(2).alongWith(intakePivotSubsystem.intakePivot(-3)));
    NamedCommands.registerCommand("AlgaeL3", elevator.elevatorUp(3).alongWith(intakePivotSubsystem.intakePivot(-3)));

    NamedCommands.registerCommand("GroundAlgae", intakePivotSubsystem.intakePivot(-2).andThen(elevator.elevatorDown()).andThen(intake.runIntakesAuto(-0.3)));


    NamedCommands.registerCommand("Pivot to Station", intakePivotSubsystem.intakePivot(-1));
    NamedCommands.registerCommand("Pivot to L2L3", intakePivotSubsystem.intakePivot(0));
    NamedCommands.registerCommand("Pivot to L1", intakePivotSubsystem.intakePivot(-1));

    FollowPathCommand.warmupCommand().schedule();

    autoChooser = AutoBuilder.buildAutoChooser();

    // UsbCamera camera = CameraServer.startAutomaticCapture();
    // camera.setResolution(160, 120);
    // camera.setFPS(24);

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
    SmartDashboard.putBoolean("Sensorrobot:", coralSensor.get()); 



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

      //if intaking or outaking, set colors depending on direction
      if(intake.getIntakeSpeed() != 0){
        if(intake.getIntakeSpeed() > 0){
          for (int i = 0; i < 94; i += 2) {
            m_ledBuffer.setRGB(i, 255, 255, 0); // grb
          }
        } else{
          for (int i = 0; i < 94; i += 2) {
            m_ledBuffer.setRGB(i, 0, 80, 80); // grb
          }
        }
      } 

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
    // placing coral
    Trigger elevatorLevelFour = operatorController.y().and(operatorController.povLeft().negate());
    Trigger elevatorLevelThree = operatorController.x().and(operatorController.povLeft().negate());
    Trigger elevatorLevelTwo = operatorController.b().and(operatorController.povLeft().negate());
    Trigger elevatorLevelOne = operatorController.a().and(operatorController.povLeft().negate());
    Trigger elevatorDown = operatorController.povDown();

    Trigger humanStation = operatorController.start();

//Trigger bargeScore = operatorController.rightStick();

    // intake and out
    Trigger intakeIn = operatorController.rightBumper().and(operatorController.povLeft().negate());
    Trigger intakeInOverride = operatorController.rightBumper().and(operatorController.rightTrigger());
    Trigger intakeOut = operatorController.leftBumper().and(operatorController.povLeft().negate());
     intakeAlgaeIn = operatorController.rightBumper().and(operatorController.povCenter().negate());
    Trigger intakeAlgaeOut = operatorController.leftBumper().and(operatorController.povCenter().negate());

    // harvesting algae
   // Trigger algaeGroundIntake = operatorController.povLeft().and(operatorController.a());
    // Trigger algaeLevelOneIntake = operatorController.povLeft().and(operatorController.b());
    // Trigger algaeLevelTwoIntake = operatorController.povLeft().and(operatorController.x());
Trigger algaeGroundIntake = operatorController.povCenter().negate().and(operatorController.povDown().negate()).and(operatorController.a());
Trigger algaeLevelOneIntake = operatorController.povCenter().negate().and(operatorController.povDown().negate()).and(operatorController.b());
Trigger algaeLevelTwoIntake = operatorController.povCenter().negate().and(operatorController.povDown().negate()).and(operatorController.x());

//boolean x, b;
//DriveCommand driveCommandInstance = new DriveCommand(swerve, driverController, operatorController, limelight, camera);
Trigger test = new Trigger(() -> (drivetrain.isDoneMoving() && elevator.moveInPosistion() && intakePivotSubsystem.moveInPosistion()));
test.whileTrue(new RunCommand(() -> intake.runIntakes(0.10), intake).andThen(new RunCommand(() -> intake.runIntakes(0.0), intake)));

    //
// +=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=
    // Bindings
    // coral

    
    //bargeScore.whileTrue(elevator.elevatorUp(3).alongWith(intakePivotSubsystem.intakePivot(6)).andThen(intakePivotSubsystem.intakePivot(-2).raceWith(new WaitCommand(0.55)).andThen(intake.runIntakes(-1))));
    elevatorLevelFour.whileTrue(elevator.elevatorUp(4).alongWith(intakePivotSubsystem.intakePivot(6.25)));
    elevatorLevelThree.whileTrue(elevator.elevatorUp(3).alongWith(intakePivotSubsystem.intakePivot(5.5)));
    elevatorLevelTwo.whileTrue(elevator.elevatorUp(2).alongWith(intakePivotSubsystem.intakePivot(5.5)));
    elevatorLevelOne.whileTrue(elevator.elevatorUp(1).alongWith(intakePivotSubsystem.intakePivot(0)));
    elevatorDown.whileTrue(elevator.elevatorDown().alongWith(intakePivotSubsystem.intakePivot(0)));

    humanStation.whileTrue(elevator.elevatorUp(5).alongWith(intakePivotSubsystem.intakePivot(1.8)));

    // algae
    algaeGroundIntake.whileTrue(elevator.elevatorDown().alongWith(intakePivotSubsystem.intakePivot(-3.8)));
    algaeLevelOneIntake.whileTrue(elevator.elevatorUp(2).alongWith(intakePivotSubsystem.intakePivot(-2.7)));
    algaeLevelTwoIntake.whileTrue(elevator.elevatorUp(3).alongWith(intakePivotSubsystem.intakePivot(-2.7)));

    // intake and out
    // intakeIn.onTrue(intake.runIntakes(0.10));
    intakeOut.whileTrue(intake.runIntakes(-0.10));
    intakeIn.whileTrue(intake.runIntakes(0.10).until( ()-> coralSensor.get()));
    intakeInOverride.whileTrue(intake.runIntakes(0.10));
    // .until( ()-> intake.coralSensor.get() == true)
    // .until(intake.coralSensor::get)
    intakeAlgaeIn.whileTrue(intake.reverseIntakes(0.4));
    intakeAlgaeOut.whileTrue(intake.runIntakes(-0.24));

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
