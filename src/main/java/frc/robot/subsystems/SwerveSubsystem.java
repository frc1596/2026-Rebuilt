package frc.robot.subsystems;

import org.deceivers.swerve.SwerveDrive;
import org.deceivers.swerve.SwerveModuleV3;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.spark.SparkMax;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveSubsystem extends SubsystemBase {
  // private final SparkMax mDriveMotor1 = new SparkMax(2, MotorType.kBrushless);
  // private final SparkMax mDriveMotor2 = new SparkMax(4, MotorType.kBrushless);
  // private final SparkMax mDriveMotor3 = new SparkMax(6, MotorType.kBrushless);
  // private final SparkMax mDriveMotor4 = new SparkMax(8, MotorType.kBrushless);

  private final TalonFX mDriveMotor1 = new TalonFX(2);
  private final TalonFX mDriveMotor2 = new TalonFX(4);
  private final TalonFX mDriveMotor3 = new TalonFX(6);
  private final TalonFX mDriveMotor4 = new TalonFX(8);
  
    // TalonFXConfiguration driveConfig = new TalonFXConfiguration();
  
    // mDriveMotor1.getConfigurator().apply(driveConfig);
    // mDriveMotor2.getConfigurator().apply(driveConfig);
    // mDriveMotor3.getConfigurator().apply(driveConfig);
    // mDriveMotor4.getConfigurator().apply(driveConfig);

  private final SparkMax mAzimuth1 = new SparkMax(1, MotorType.kBrushless);
  private final SparkMax mAzimuth2 = new SparkMax(3, MotorType.kBrushless);
  private final SparkMax mAzimuth3 = new SparkMax(5, MotorType.kBrushless);
  private final SparkMax mAzimuth4 = new SparkMax(7, MotorType.kBrushless);

  private final CANcoder mAzimuthEncoder1 = new CANcoder(9); 
  private final CANcoder mAzimuthEncoder2 = new CANcoder(10); 
  private final CANcoder mAzimuthEncoder3 = new CANcoder(11);  
  private final CANcoder mAzimuthEncoder4 = new CANcoder(12); 

  //corners are done like a graph. Positive x,y in quadrant 1, and so on
  private final SwerveModuleV3 Module1 = new SwerveModuleV3(mAzimuth1, mDriveMotor1, new Translation2d(-0.352425, 0.200025), "Module 1", mAzimuthEncoder1);
  private final SwerveModuleV3 Module2 = new SwerveModuleV3(mAzimuth2, mDriveMotor2, new Translation2d(0.352425, 0.200025), "Module 2", mAzimuthEncoder2);
  private final SwerveModuleV3 Module3 = new SwerveModuleV3(mAzimuth3, mDriveMotor3, new Translation2d(0.352425, -0.200025), "Module 3", mAzimuthEncoder3);
  private final SwerveModuleV3 Module4 = new SwerveModuleV3(mAzimuth4, mDriveMotor4, new Translation2d(-0.352425,  -0.200025), "Module 4", mAzimuthEncoder4);

  //auto align stuff
 private PIDController xController = new PIDController(0.3, 0, 0.0); // left right
  private PIDController yController = new PIDController(0.2, 0.0, 0.0);// forward backward
  private PIDController autoAimController = new PIDController(0.3, 0, 0.0);
  private LinearFilter aimFilter = LinearFilter.movingAverage(3);
  private LinearFilter xFilter2 = LinearFilter.movingAverage(3);
  private LinearFilter yFilter2 = LinearFilter.movingAverage(3);
  private SlewRateLimiter xfilter = new SlewRateLimiter(3);
  private SlewRateLimiter yfilter = new SlewRateLimiter(3);


  private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
int limelightlimiter = 0;
  private final SwerveDrive mSwerveDrive = new SwerveDrive(this::getRotation, Module1, Module2, Module3, Module4);
  private SimSwerveModule[] modules;
  private SwerveDriveKinematics kinematics;
  private SwerveDriveOdometry odometry;
  private Limelight limelight = new Limelight();
  // Vision stuff
  // private final PhotonCamera camera = new PhotonCamera("OV9281");
  // public static final AprilTagFieldLayout kTagLayout = new AprilTagFieldLayout("2025/championship.json");
  // private final PhotonPoseEstimator photonEstimator = new PhotonPoseEstimator(camera, PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY, kTagLayout, new Pose2d(0, 0, new Rotation2d()));

  //private final PhotonPoseEstimator photonEstimator = new PhotonPoseEstimator(camera, PhotonPoseEstimator.PoseStrategy.AVERAGE_BEST_TARGETS, kTagLayout, new Pose2d(0, 0, new Rotation2d()));

  public SwerveSubsystem() {
   gyro.reset();

   //pathplanner config,
   final RobotConfig ppConfig =
      new RobotConfig(
          70.0,
          6.8,
          new ModuleConfig(
              0.0508,
              5.06,
              1,
              DCMotor.getFalcon500(1), 
              6.12,
              40,
              1),
              new Translation2d[] {
                new Translation2d(0.29845, -0.29845),
                new Translation2d(-0.29845, -0.29845),
                new Translation2d(-0.29845, 0.29845),
                new Translation2d(0.29845, 0.29845)
              });


    // Configure AutoBuilder last
    AutoBuilder.configure(
            mSwerveDrive::getPose, // Robot pose supplier
            mSwerveDrive::resetPosePathplanner, // Method to reset odometry (will be called if your auto has a starting pose)
            mSwerveDrive::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> mSwerveDrive.drivePathplanner(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(1, 0.0, 0), // Translation PID constants 3.8
                    new PIDConstants(1, 0.0, 0) // Rotation PID constants 3.5
            ),ppConfig,
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
  );
  }

  @Override
  public void periodic() {
   // odometry.update(gyro.getRotation2d(), getPositions());
  //  if((limelightlimiter%500) == 0){
  //     mSwerveDrive.updatePoseWithVision(limelight.getPose(), limelight.getTimeStamp());
  //  }
    //mSwerveDrive.updateOdometry();

  //SmartDashboard.putNumber("RobotXSpeed", mSwerveDrive.getChassisSpeeds().vxMetersPerSecond);
  //SmartDashboard.putNumber("RobotYSpeed", mSwerveDrive.getChassisSpeeds().vyMetersPerSecond);
  //SmartDashboard.putNumber("RobotXPos", mSwerveDrive.getPose().getX());
  //SmartDashboard.putNumber("RobotYPos", mSwerveDrive.getPose().getY());

  mSwerveDrive.periodic(); 
  }
  public void drive(double forward, double strafe, double azimuth, boolean fieldRelative){
      azimuth = azimuth*2.5;
      mSwerveDrive.drive(forward, strafe, azimuth, fieldRelative);
    }
  
    public void driveClosedLoop(double forward, double strafe, double azimuth, boolean fieldRelative){
      if (!fieldRelative){
        forward = -forward;
        strafe = -strafe;
      }
      azimuth = azimuth*2.5;
      mSwerveDrive.driveClosedLoop(forward, strafe, azimuth, fieldRelative);
    }
  
    public void stop(){
      mSwerveDrive.stop();
    }
  
    public void setLocation(double x, double y, double angle){
      mSwerveDrive.setLocation(x, y, angle);
    }

    public void setModulesAngle(double angle, int module){
     // mSwerveDrive.setModulesAngle(angle, module);
    }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }
  public double getRotation() {
    //SmartDashboard.putNumber("Gyro", -gyro.getAngle());
    return -gyro.getAngle();
 }
  public void resetPose(Pose2d pose) {
    odometry.resetPosition(new Rotation2d(gyro.getYaw()), getPositions(), pose);
  }

  public ChassisSpeeds getSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, new Rotation2d(gyro.getYaw())));
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    //ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    ChassisSpeeds targetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(robotRelativeSpeeds, new Rotation2d(gyro.getYaw()));

    SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
    setStates(targetStates);
  }

  public void resetGyro(){
    gyro.reset();
  }

  public void setStates(SwerveModuleState[] targetStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, 3.81);

    for (int i = 0; i < modules.length; i++) {
      modules[i].setTargetState(targetStates[i]);
      
    }
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }




  /**
   * Basic simulation of a swerve module, will just hold its current state and not use any hardware
   */
  class SimSwerveModule {
    private SwerveModulePosition currentPosition = new SwerveModulePosition();
    private SwerveModuleState currentState = new SwerveModuleState();

    public SwerveModulePosition getPosition() {
      return currentPosition;
    }

    public SwerveModuleState getState() {
      return currentState;
    }

    public void setTargetState(SwerveModuleState targetState) {
      // Optimize the state
      currentState = SwerveModuleState.optimize(targetState, currentState.angle);

      currentPosition = new SwerveModulePosition(currentPosition.distanceMeters + (currentState.speedMetersPerSecond * 0.02), currentState.angle);
    }
  }

    public Command alignCommand(){
      return this.runEnd(() -> {
        double yVel = 0;
        double xVel = 0; 
        double rotVel = 0;

        if (LimelightHelpers.getTV("limelight")) {
          yVel = -yController
          .calculate(xFilter2.calculate(LimelightHelpers.getCameraPose3d_TargetSpace("limelight").getZ()), -0.6); // forward
                                                                                                                  // backward
      xVel = xController.calculate(yFilter2.calculate(LimelightHelpers.getCameraPose3d_TargetSpace("limelight").getX()),
          0.18);
      rotVel = autoAimController.calculate(
          aimFilter.calculate(LimelightHelpers.getCameraPose3d_TargetSpace("limelight").getRotation().getY()), 0.08);
        
          drive(yfilter.calculate(yVel), xfilter.calculate(xVel), rotVel, true);
        }


      }, () -> Commands.waitSeconds(1));
    }
}