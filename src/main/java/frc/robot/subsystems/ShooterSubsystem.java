package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
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
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.util.Interpolable;
import frc.robot.util.InterpolatingDouble;


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
    //private final SparkClosedLoopController mrotatePID;


    private final TrapezoidProfile m_pivotProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(60, 200));
    private TrapezoidProfile.State m_hoodGoal = new TrapezoidProfile.State(0,0); 
    private TrapezoidProfile.State m_hoodsetpoint = new TrapezoidProfile.State(0,0);
 

    // private TrapezoidProfile.State m_rotategoal = new TrapezoidProfile.State(0,0); 
    // private TrapezoidProfile.State m_rotatesetpoint = new TrapezoidProfile.State(0,0);
 private final Constraints m_rotateprofile = new Constraints(.05, 0);

    LinearFilter filter = LinearFilter.movingAverage(5);
    LinearFilter shootfilter = LinearFilter.movingAverage(5);
    LinearFilter rotatefilter = LinearFilter.movingAverage(5);

    // private final VisionSubsytem vision;

private ProfiledPIDController autoaimController = new ProfiledPIDController(0.01, 0, 0, m_rotateprofile);

    private static InterpolatingDoubleTreeMap hoodmap = new InterpolatingDoubleTreeMap();
     private static InterpolatingDoubleTreeMap shootmap = new InterpolatingDoubleTreeMap();


    CommandXboxController moperatorController;
    VisionSubsytem mvision;
    public ShooterSubsystem(VisionSubsytem vision, CommandXboxController operatorController) {
        moperatorController = operatorController;
        mvision = vision;
        // this.vision = vision;
        SparkMaxConfig spindexterConfig = new SparkMaxConfig();
        SparkMaxConfig turretHoodConfig = new SparkMaxConfig();
        SparkMaxConfig turretRotateConfig = new SparkMaxConfig();
        spindexterConfig.idleMode(IdleMode.kCoast);
        spindexterConfig.encoder.positionConversionFactor(1);
        spindexterConfig.encoder.velocityConversionFactor(1);
        spindexterConfig.smartCurrentLimit(50);
        spindexterConfig.inverted(false);
        spindexter.configure(spindexterConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
       
        turretHoodConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pidf(1.6,0,0,0); //Deprecated. Use ClosedLoopConfig.feedForward to set feedforward gains
        turretHoodConfig.idleMode(IdleMode.kBrake);
        turretHoodConfig.encoder.positionConversionFactor(1);
        turretHoodConfig.encoder.velocityConversionFactor(1);
        turretHoodConfig.smartCurrentLimit(30);
        turretHoodConfig.inverted(false);
        turretHood.configure(turretHoodConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
        mHoodEncoder = turretHood.getEncoder();
        mHoodEncoder.setPosition(0); 
        mHoodPID = turretHood.getClosedLoopController(); 

        //turretRotateConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pidf(1.0,0,0,0); //Deprecated. Use ClosedLoopConfig.feedForward to set feedforward gains
        turretRotateConfig.idleMode(IdleMode.kBrake);
        turretRotateConfig.encoder.positionConversionFactor(1);
        turretRotateConfig.encoder.velocityConversionFactor(1);
        turretRotateConfig.smartCurrentLimit(15);
        turretRotateConfig.inverted(false);
        turretRotate.configure(turretHoodConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
        mrotateencoder = turretRotate.getEncoder();
        mrotateencoder.setPosition(0); 
        //mrotatePID = turretRotate.getClosedLoopController(); 

        hoodmap.put(4.5,3.0);// ~6 ft
        hoodmap.put(4.0, 2.5);// ~9ft
        hoodmap.put(3.0, 2.0);// ~12 ft
        hoodmap.put(2.4, 1.4);
        hoodmap.put(2.0,1.2);// ~15 ft
        hoodmap.put(1.0,1.0);// ~15 ft
        hoodmap.put(0.0,0.0);// ~15 ft

        shootmap.put(4.0, 80.0);// ~9ft
        shootmap.put(3.0, 70.0);// ~12 ft
        shootmap.put(2.4, 60.0);
        shootmap.put(2.0,55.0);// ~15 ft
        shootmap.put(1.0,50.0);// ~15 ft
        shootmap.put(0.0,45.0);// ~15 ft

        TalonFXConfiguration feederConfig = new TalonFXConfiguration();
        feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        feederConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        feederConfig.CurrentLimits.SupplyCurrentLimit = 40;
        feeder.getConfigurator().apply(feederConfig, 0.05);

        TalonFXConfiguration shootoneConfig = new TalonFXConfiguration();
        shootoneConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        shootoneConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        shootoneConfig.CurrentLimits.SupplyCurrentLimit = 40;
        shootoneConfig.Slot0.kV = .1;
        shootoneConfig.Slot0.kP = .3;
        shootoneConfig.Slot0.kI = 0;
        shootoneConfig.Slot0.kD = 0;
        shootOne.getConfigurator().apply(shootoneConfig, 0.05);

        TalonFXConfiguration shoottwoConfig = new TalonFXConfiguration();
        shoottwoConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        shoottwoConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        shoottwoConfig.CurrentLimits.SupplyCurrentLimit = 40;
        shoottwoConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shoottwoConfig.Slot0.kV = .1;
        shoottwoConfig.Slot0.kP = .3;
        shoottwoConfig.Slot0.kI = 0;
        shoottwoConfig.Slot0.kD = 0;
        shootTwo.getConfigurator().apply(shoottwoConfig, 0.05);
    }
    
    @Override
    public void periodic() {
        //LimelightHelpers.getTV("limelte");
        Pose3d position = LimelightHelpers.getCameraPose3d_TargetSpace("limelight");
      
        autoaimController.setGoal(0);
        autoaimController.calculate(LimelightHelpers.getTX("limelight"));
        // Transform3d pose = LimelightHelpers.getBotPose3d_TargetSpace("limelight");
        double distance = Math.sqrt(position.getZ()*position.getZ()+position.getX()*position.getX());
        double hoodAngle = hoodmap.get(filter.calculate(distance));
                double shooterSpeed = shootmap.get(shootfilter.calculate(distance));
        double rotateangle = Math.atan2(-position.getZ(), position.getX());
        rotateTurret(autoaimController.calculate(LimelightHelpers.getTX("limelight")));


        m_hoodsetpoint = m_pivotProfile.calculate(.02, m_hoodsetpoint, m_hoodGoal);
       // m_rotatesetpoint = m_rotateprofile.calculate(.02, m_rotatesetpoint, m_rotategoal);

SmartDashboard.putNumber("Dist", distance);
SmartDashboard.putNumber("Hoodangle", hoodAngle);
SmartDashboard.putNumber("rotate", rotateangle);



        //Set new position to the PID Controller
     mHoodPID.setReference(m_hoodsetpoint.position, com.revrobotics.spark.SparkBase.ControlType.kPosition); 
    //mrotatePID.setReference(m_rotatesetpoint.position, com.revrobotics.spark.SparkBase.ControlType.kPosition); 


    SmartDashboard.putNumber("bob", getShootOneSpeed());

        if (moperatorController.rightBumper().getAsBoolean()) {
                setHoodAngle(hoodAngle);

            if (getShootOneSpeed() == 0.0 && getShootTwoSpeed() == 0.0) // if the shooters have not been started
            {
                setShootSpeed(shooterSpeed); // start the shooter
            } else if (Math.abs(getShootOneSpeed()) > (shooterSpeed-5.0)) // if the shooters
                                                                                                   // are up to speed
            {
                setSpindexterSpeed(-1.0); // start everything else
                setFeederSpeed(1.0);
                setShootSpeed(shooterSpeed); // start the shooter
            } else {
                // doNothing();
            }
        }else {
            setSpindexterSpeed(0);
            setFeederSpeed(0);
                            setHoodAngle(0);

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

    public void rotateTurret(double angle) {
        //m_rotategoal = new TrapezoidProfile.State(angle, 0);  
        turretRotate.set(angle);
      }

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

    public double getShootTwoSpeed() {
        SmartDashboard.putNumber("ShooterSpeed", getShootOneSpeed());
        return (shootTwo.getRotorVelocity().getValueAsDouble());
    }

    // it starts the shoot
    public void startShoot() {


        // if (getShootOneSpeed() == 0.0 && getShootTwoSpeed() == 0.0) // if the
        // shooters have not been started
        // {
        // setShootSpeed(0.3); // start the shooter
        // }
        // else if (Math.abs(getShootOneSpeed()) > 0.1 && (Math.abs(getShootTwoSpeed())
        // > 0.1)) // if the shooters are up to speed
        // {
        // setSpindexterSpeed(-.4); // start everything else
        // setFeederSpeed(.1);
        // }
        // else{
        // //doNothing();
        // }
    }

    public Command shootCommand() {
        return this.startEnd(() -> startShoot(), () -> stopshoot());
    }
}
