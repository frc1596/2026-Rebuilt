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

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
    private final SparkClosedLoopController mHoodPID;

    private final TrapezoidProfile m_pivotProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(60, 200));
    private TrapezoidProfile.State m_hoodGoal = new TrapezoidProfile.State(0,0); 
    private TrapezoidProfile.State m_hoodsetpoint = new TrapezoidProfile.State(0,0);
 
    // private final VisionSubsytem vision;

    // private static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> shootMap = new InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>(
    //         null, null);
    // static {
    //     shootMap.put(new InterpolatingDouble(0.0), new InterpolatingDouble(45.0));// ~6 ft
    //     shootMap.put(new InterpolatingDouble(1.0), new InterpolatingDouble(50.0));// ~9ft
    //     shootMap.put(new InterpolatingDouble(2.0), new InterpolatingDouble(55.0));// ~12 ft
    //     shootMap.put(new InterpolatingDouble(3.0), new InterpolatingDouble(60.0));// ~15 ft
    //     shootMap.put(new InterpolatingDouble(4.0), new InterpolatingDouble(65.0));// ~18 ft
    //}
    private static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> hoodmap = new InterpolatingTreeMap<>(
            null, null);
    static {
        hoodmap.put(new InterpolatingDouble(5.0), new InterpolatingDouble(-3.0));// ~6 ft
        hoodmap.put(new InterpolatingDouble(4.0), new InterpolatingDouble(-2.5));// ~9ft
        hoodmap.put(new InterpolatingDouble(3.0), new InterpolatingDouble(-2.0));// ~12 ft
        hoodmap.put(new InterpolatingDouble(2.0), new InterpolatingDouble(-1.5));// ~15 ft
        hoodmap.put(new InterpolatingDouble(1.0), new InterpolatingDouble(-1.0));
        hoodmap.put(new InterpolatingDouble(0.0), new InterpolatingDouble(-0.4));// ~18 ft
    }

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
        shoottwoConfig.CurrentLimits.SupplyCurrentLimit = 20;
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
        Pose3d position = LimelightHelpers.getCameraPose3d_TargetSpace("limelight-limelte");
       
        // Transform3d pose = LimelightHelpers.getBotPose3d_TargetSpace("limelight");


        double hoodAngle = -hoodmap.get(new InterpolatingDouble(-position.getZ())).value;
        
    //setHoodAngle(hoodAngle);
        //m_hoodsetpoint = m_pivotProfile.calculate(.02, m_hoodsetpoint, m_hoodGoal);
SmartDashboard.putNumber("ZDist", position.getZ());
SmartDashboard.putNumber("Hoodangle", hoodAngle);

        //Set new position to the PID Controller
     //mHoodPID.setReference(m_hoodsetpoint.position, com.revrobotics.spark.SparkBase.ControlType.kPosition); 
   
    SmartDashboard.putNumber("bob", getShootOneSpeed());

        if (moperatorController.rightBumper().getAsBoolean()) {
            if (getShootOneSpeed() == 0.0 && getShootTwoSpeed() == 0.0) // if the shooters have not been started
            {
                setShootSpeed(65); // start the shooter
            } else if (Math.abs(getShootOneSpeed()) > 60.0 && (Math.abs(getShootTwoSpeed()) > 60.0)) // if the shooters
                                                                                                   // are up to speed
            {
                setSpindexterSpeed(-1.0); // start everything else
                setFeederSpeed(1.0);
                setShootSpeed(65); // start the shooter
            } else {
                // doNothing();
            }
        }else {
            setSpindexterSpeed(0);
            setFeederSpeed(0);
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
