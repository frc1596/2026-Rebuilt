package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorOutputStatusValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.util.Interpolable;
import frc.robot.util.InterpolatingDouble;

import static edu.wpi.first.units.Units.Rotation;

import java.security.cert.TrustAnchor;

import javax.print.attribute.standard.Media;

import org.deceivers.swerve.SwerveDrive;

public class ShooterSubsystem extends SubsystemBase {

    private final SparkMax spindexter = new SparkMax(51, MotorType.kBrushless);
    private final SparkMax turretHood = new SparkMax(50, MotorType.kBrushless);
    private final SparkMax turretRotate = new SparkMax(53, MotorType.kBrushless);
    private final TalonFX feeder = new TalonFX(20);
    private final TalonFX shootOne = new TalonFX(54);
    private final TalonFX shootTwo = new TalonFX(55);
    private final VelocityVoltage m_VelocityVoltage = new VelocityVoltage(0);
    private final RelativeEncoder mHoodEncoder;
    private final RelativeEncoder mrotateencoder;
    private final SparkClosedLoopController mHoodPID;
    private final SparkClosedLoopController mrotatePID;


    private final TrapezoidProfile m_pivotProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(60, 200));
    private TrapezoidProfile.State m_hoodGoal = new TrapezoidProfile.State(0,0); 
    private TrapezoidProfile.State m_hoodsetpoint = new TrapezoidProfile.State(0,0);
 

     private TrapezoidProfile.State m_rotategoal = new TrapezoidProfile.State(0,0); 
     private TrapezoidProfile.State m_rotatesetpoint = new TrapezoidProfile.State(0,0);
 private final TrapezoidProfile m_rotateprofile =  new TrapezoidProfile(new TrapezoidProfile.Constraints(1500, 5000));

    // MedianFilter filter = new MedianFilter(19);
    // MedianFilter shootfilter = new MedianFilter(19);
    // MedianFilter rotatefilter = new MedianFilter(19);

    LinearFilter filter = LinearFilter.movingAverage(5);
    LinearFilter shootfilter = LinearFilter.movingAverage(5);
   // LinearFilter rotatefilter = LinearFilter.movingAverage(5);
    // private final VisionSubsytem vision;
 private final Constraints limelight_rotateprofile = new Constraints(.05, 0);
 private ProfiledPIDController limelightautoaimController = new ProfiledPIDController(0.015, 0, 0, limelight_rotateprofile);

    private static InterpolatingDoubleTreeMap hoodmap = new InterpolatingDoubleTreeMap();
     private static InterpolatingDoubleTreeMap shootmap = new InterpolatingDoubleTreeMap();
     private static InterpolatingDoubleTreeMap tof = new InterpolatingDoubleTreeMap();

      Transform2d shootertransform = new Transform2d(9.375/39.37, 3.5/39.37, Rotation2d.fromDegrees(180));

     CommandXboxController moperatorController;
     VisionSubsytem mvision;
     SwerveSubsystem mSwerve; 
     Field mField;

     public ShooterSubsystem(CommandXboxController operatorController, SwerveSubsystem swerve,Field Field1) {
        hoodmap.put(5.5,3.0);// ~6 ft
        hoodmap.put(5.0, 2.5);// ~9ft
        hoodmap.put(4.0, 2.0);// ~12 ft
        hoodmap.put(3.0,1.5);// ~15 ft
        hoodmap.put(2.0,1.1);// ~15 ft
        hoodmap.put(1.0,0.0);// ~15 ft
        hoodmap.put(0.0,0.0);// ~15 ft

        shootmap.put(12.0,100.0);
        shootmap.put(7.0,80.0);
        shootmap.put(5.5, 68.0);// ~9ft
        shootmap.put(5.0, 63.0);// ~9ft
        shootmap.put(4.0, 57.0);// ~12 ft good
        shootmap.put(3.0,53.0);// ~15 ft
        shootmap.put(2.0,45.0);// ~15 ft
        shootmap.put(1.0,45.0);// ~15 ft
        shootmap.put(0.0,30.0);// ~15 ft

        tof.put(0.0, 0.8);
        tof.put(4.5, 2.0);

        moperatorController = operatorController;
       /// mvision = vision;
        mSwerve = swerve; 
        mField = Field1;
        // this.vision = vision;
        SparkMaxConfig spindexterConfig = new SparkMaxConfig();
        SparkMaxConfig turretHoodConfig = new SparkMaxConfig();
        SparkMaxConfig turretRotateConfig = new SparkMaxConfig();
        spindexterConfig.idleMode(IdleMode.kCoast);
        spindexterConfig.encoder.positionConversionFactor(1);
        spindexterConfig.encoder.velocityConversionFactor(1);
        spindexterConfig.smartCurrentLimit(40);
        spindexterConfig.inverted(false);
        spindexter.configure(spindexterConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
        
        turretHoodConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pidf(1.8,0,0,0); //Deprecated. Use ClosedLoopConfig.feedForward to set feedforward gains
        turretHoodConfig.idleMode(IdleMode.kBrake);
        turretHoodConfig.encoder.positionConversionFactor(1);
        turretHoodConfig.encoder.velocityConversionFactor(1);
        turretHoodConfig.smartCurrentLimit(40);
        turretHoodConfig.inverted(false);
        turretHood.configure(turretHoodConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
        mHoodEncoder = turretHood.getEncoder();
        mHoodEncoder.setPosition(0); 
        mHoodPID = turretHood.getClosedLoopController(); 
        
        turretRotateConfig.softLimit.forwardSoftLimit(190);
        turretRotateConfig.softLimit.reverseSoftLimit(-190);
        turretRotateConfig.softLimit.forwardSoftLimitEnabled(true);
        turretRotateConfig.softLimit.reverseSoftLimitEnabled(true);
        turretRotateConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pidf(0.12,0,0,0); //Deprecated. Use ClosedLoopConfig.feedForward to set feedforward gains
  //  turretRotateConfig.closedLoop.allowedClosedLoopError(1.0, ClosedLoopSlot.kSlot0);

        turretRotateConfig.idleMode(IdleMode.kBrake);
        turretRotateConfig.encoder.positionConversionFactor((24.0/115.0)*(1.0/9.0)*360);//degrees
        turretRotateConfig.encoder.velocityConversionFactor((24.0/115.0)*(1.0/9.0)*360);//degrees per minute
        turretRotateConfig.smartCurrentLimit(40);
        turretRotateConfig.inverted(false);
        turretRotate.configure(turretRotateConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
        mrotateencoder = turretRotate.getEncoder();
        mrotateencoder.setPosition(0); 
        mrotatePID = turretRotate.getClosedLoopController();


        TalonFXConfiguration feederConfig = new TalonFXConfiguration();
        feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        feederConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        feederConfig.CurrentLimits.SupplyCurrentLimit = 40;
        feeder.getConfigurator().apply(feederConfig, 0.05);

        TalonFXConfiguration shootoneConfig = new TalonFXConfiguration();
        shootoneConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        shootoneConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        shootoneConfig.CurrentLimits.SupplyCurrentLimit = 60;
        shootoneConfig.CurrentLimits.SupplyCurrentLowerTime = 1;
        shootoneConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
        shootoneConfig.Slot0.kV = .11;
        shootoneConfig.Slot0.kP = .36;
        shootoneConfig.Slot0.kS = 0.14;
        shootoneConfig.Slot0.kI = 0;
        shootoneConfig.Slot0.kD = 0;
        shootOne.getConfigurator().apply(shootoneConfig, 0.05);

        TalonFXConfiguration shoottwoConfig = new TalonFXConfiguration();
        shoottwoConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        shoottwoConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        shoottwoConfig.CurrentLimits.SupplyCurrentLimit = 60;
        shoottwoConfig.CurrentLimits.SupplyCurrentLowerTime = 1;
        shoottwoConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
        shoottwoConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shoottwoConfig.Slot0.kV = .11;
        shoottwoConfig.Slot0.kP = .36;
        shoottwoConfig.Slot0.kI = 0;
        shoottwoConfig.Slot0.kS = 0.14;
        shoottwoConfig.Slot0.kD = 0;
        shootTwo.getConfigurator().apply(shoottwoConfig, 0.05);
    }

    public double prevangle = 0;
    public double distance = 0;
    private boolean inAuto = false;

    @Override
    public void periodic() {


        double hoodAngle = 0;
        double shooterSpeed = 0;

        Pose2d swervexy = mSwerve.mSwerveDrive.getPose();
        Pose2d turretxy = swervexy.transformBy(shootertransform);

        double turretAngle;
        double shootangle = 0;

        SmartDashboard.putNumber("TurretX", turretxy.getX());
        SmartDashboard.putNumber("TurretY", turretxy.getY());

        //check which alliance we're on
        var alliance = DriverStation.getAlliance();

        if (alliance.isPresent()) {
            if(moperatorController.leftBumper().getAsBoolean()){ //if pass
                if(alliance.get() == DriverStation.Alliance.Red){
                    if(turretxy.getY() < Field.redHubCenter.getY()){
                        distance = mField.feedRedLeft.getDistance(turretxy.getTranslation());
                        shootangle = mField.feedRedLeft.minus(turretxy.getTranslation()).getAngle().getDegrees();
                    }else{
                        distance = mField.feedRedRight.getDistance(turretxy.getTranslation());
                        shootangle = mField.feedRedRight.minus(turretxy.getTranslation()).getAngle().getDegrees();
                    }
                }else{
                    if(turretxy.getY() > Field.blueHubCenter.getY()){
                        distance = mField.feedBlueLeft.getDistance(turretxy.getTranslation());
                        shootangle = mField.feedBlueLeft.minus(turretxy.getTranslation()).getAngle().getDegrees();
                    }else{
                        distance = mField.feedBlueRight.getDistance(turretxy.getTranslation());
                        shootangle = mField.feedBlueRight.minus(turretxy.getTranslation()).getAngle().getDegrees();
                    }
                }
            }
            else if(alliance.get() == DriverStation.Alliance.Red){
                //aim for red hub if on red alliance
                distance = mField.redHubCenter.toTranslation2d().getDistance(turretxy.getTranslation());
                shootangle = mField.redHubCenter.toTranslation2d().minus(turretxy.getTranslation()).getAngle().getDegrees();       
            }else {
                //aim for blue hub if on blue alliance
                distance = mField.blueHubCenter.toTranslation2d().getDistance(turretxy.getTranslation());
                shootangle = mField.blueHubCenter.toTranslation2d().minus(turretxy.getTranslation()).getAngle().getDegrees();
            }
        }

        SmartDashboard.putNumber("Hubdistance", distance);

        //SWAP BACK IF NEEDED
        //calculate and set turret angle
       // turretAngle = (-shootangle-90.0 + mSwerve.getRotation());
         turretAngle = (-shootangle - 90.0 + mSwerve.mSwerveDrive.mSwerveDrivePoseEstimator.getEstimatedPosition().getRotation().getDegrees());

//if left trigger pressed, use the limelight (needs tuning)
    if(moperatorController.leftTrigger(0.5).getAsBoolean() && !moperatorController.leftBumper().getAsBoolean()){
        Pose3d limelightposition = LimelightHelpers.getCameraPose3d_TargetSpace("limelight");
        limelightautoaimController.setGoal(0);
        limelightautoaimController.calculate(LimelightHelpers.getTX("limelight"));

        //if Limelight has a valid target,
        if(LimelightHelpers.getTV("limelight")){
            distance = Math.sqrt(limelightposition.getZ()*limelightposition.getZ()+limelightposition.getX()*limelightposition.getX());
        }
    double limelightOutput = limelightautoaimController.calculate(LimelightHelpers.getTX("limelight"));
    rotateTurret(mrotateencoder.getPosition()+limelightOutput);
    }else{
        rotateTurret(Math.toDegrees(MathUtil.angleModulus(-Math.toRadians(turretAngle))));
    }
        //lookup maps
        hoodAngle = hoodmap.get(filter.calculate(distance));
        shooterSpeed = shootmap.get(shootfilter.calculate(distance));
        
        //Pid update and send to sparkmax
        m_hoodsetpoint = m_pivotProfile.calculate(.02, m_hoodsetpoint, m_hoodGoal);
        m_rotatesetpoint = m_rotateprofile.calculate(.02, m_rotatesetpoint, m_rotategoal);
        mHoodPID.setReference(m_hoodsetpoint.position, com.revrobotics.spark.SparkBase.ControlType.kPosition); 
        mrotatePID.setReference(m_rotatesetpoint.position, com.revrobotics.spark.SparkBase.ControlType.kPosition); 

        //Operator Controls
        if (moperatorController.rightBumper().getAsBoolean() || (inAuto == true)) {
            setHoodAngle(hoodAngle);

            if (getShootOneSpeed() == 0.0 && getShootTwoSpeed() == 0.0){
                setShootSpeed(shooterSpeed); // start the shooter
            } else if (Math.abs(getShootOneSpeed()) > (shooterSpeed-3.0)){
                setSpindexterSpeed(-1.0); // start everything else
                setFeederSpeed(1.0);
                setShootSpeed(shooterSpeed); // start the shooter
            } 
        }
        //pass button 
        else if (moperatorController.leftBumper().getAsBoolean()) {
            setShootSpeed(shooterSpeed); // start the shooter
            if (Math.abs(getShootOneSpeed()) > (shooterSpeed-3.0)){
                setSpindexterSpeed(-1.0); // start everything else
                setFeederSpeed(1.0);
                setShootSpeed(shooterSpeed); // start the shooter
            }
        } else{
                setSpindexterSpeed(0);
                setFeederSpeed(0);
                setHoodAngle(0);
                setShootSpeed(0);
        }



    }
    

    public void setHoodAngle(double angle)
    {
        m_hoodGoal = new TrapezoidProfile.State(angle, 0);
    }

    public void stopshoot() {
        setShootSpeed(0);
    }

    public void setSpindexterSpeed(double speed) {
        spindexter.set(-speed);
    }

    public void doNothing() {

    }
public boolean turretInPosition(){

            return Math.abs(m_rotatesetpoint.position - m_rotategoal.position) < 2.0;

}
      public void manualrotateTurret(double angle) {

       turretRotate.set(angle);
      }

    public void rotateTurret(double angle) {
        m_rotategoal = new TrapezoidProfile.State(angle, 0);  

       // turretRotate.set(angle);
      }


    // public void rotateturretleft(double angle) {
    //     turretRotate.set(angle+5);
    // }
    // public void rotateturretright(double angle){
    //     turretRotate.set(angle-5);
    // }

    public void setFeederSpeed(double speed) {
        feeder.set(-1.0 * speed);
    }

    public void setShootSpeed(double speed) {
        shootOne.setControl(m_VelocityVoltage.withVelocity(speed));
        shootTwo.setControl(m_VelocityVoltage.withVelocity(speed));
    }

    public double getShootOneSpeed() {
        return (shootOne.getRotorVelocity().getValueAsDouble());
    }
public Command startShoot(){
    //inAuto = true;
        return this.runOnce(() -> settrue());

}



public void settrue(){
    inAuto = true;
}
public void setfalse(){
    inAuto = false;
}
public Command stopShoot(){
    //inAuto = false;
        return this.runOnce(() ->setfalse());

}

    public double getShootTwoSpeed() {
        //SmartDashboard.putNumber("ShooterSpeed", getShootOneSpeed());
        return (shootTwo.getRotorVelocity().getValueAsDouble());
    }

}
