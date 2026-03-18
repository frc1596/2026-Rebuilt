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
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
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
 private final TrapezoidProfile m_rotateprofile =  new TrapezoidProfile(new TrapezoidProfile.Constraints(340, 1000));

    // MedianFilter filter = new MedianFilter(19);
    // MedianFilter shootfilter = new MedianFilter(19);
    // MedianFilter rotatefilter = new MedianFilter(19);

    LinearFilter filter = LinearFilter.movingAverage(5);
    LinearFilter shootfilter = LinearFilter.movingAverage(5);
    LinearFilter rotatefilter = LinearFilter.movingAverage(5);
    // private final VisionSubsytem vision;

//private ProfiledPIDController autoaimController = new ProfiledPIDController(0.015, 0, 0, m_rotateprofile);

    private static InterpolatingDoubleTreeMap hoodmap = new InterpolatingDoubleTreeMap();
     private static InterpolatingDoubleTreeMap shootmap = new InterpolatingDoubleTreeMap();


    CommandXboxController moperatorController;
    VisionSubsytem mvision;
    SwerveSubsystem mSwerve; 
    public ShooterSubsystem(CommandXboxController operatorController, SwerveSubsystem swerve) {
        moperatorController = operatorController;
       /// mvision = vision;
        mSwerve = swerve; 
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
        turretRotateConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pidf(0.05,0,0,0); //Deprecated. Use ClosedLoopConfig.feedForward to set feedforward gains
    turretRotateConfig.closedLoop.allowedClosedLoopError(2.5, ClosedLoopSlot.kSlot0);

        turretRotateConfig.idleMode(IdleMode.kBrake);
        turretRotateConfig.encoder.positionConversionFactor((24.0/115.0)*(1.0/5.0)*360);//degrees
        turretRotateConfig.encoder.velocityConversionFactor((24.0/115.0)*(1.0/5.0)*360);//degrees per minute
        turretRotateConfig.smartCurrentLimit(40);
        turretRotateConfig.inverted(false);
        turretRotate.configure(turretRotateConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
        mrotateencoder = turretRotate.getEncoder();
        mrotateencoder.setPosition(0); 
        mrotatePID = turretRotate.getClosedLoopController();

        hoodmap.put(4.5,3.0);// ~6 ft
        hoodmap.put(4.0, 2.5);// ~9ft
        hoodmap.put(3.0, 2.0);// ~12 ft
        hoodmap.put(2.4, 1.4);
        hoodmap.put(2.0,1.2);// ~15 ft
        hoodmap.put(1.0,1.0);// ~15 ft
        hoodmap.put(0.0,0.0);// ~15 ft

        shootmap.put(4.5, 64.0);// ~9ft
        shootmap.put(4.0, 61.0);// ~9ft
        shootmap.put(3.0, 60.0);// ~12 ft
        shootmap.put(2.4, 55.0);
        shootmap.put(2.0,50.0);// ~15 ft
        shootmap.put(1.0,45.0);// ~15 ft
        shootmap.put(0.0,40.0);// ~15 ft

        TalonFXConfiguration feederConfig = new TalonFXConfiguration();
        feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        feederConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        feederConfig.CurrentLimits.SupplyCurrentLimit = 40;
        feeder.getConfigurator().apply(feederConfig, 0.05);

        TalonFXConfiguration shootoneConfig = new TalonFXConfiguration();
        shootoneConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        shootoneConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        shootoneConfig.CurrentLimits.SupplyCurrentLimit = 70;
        shootoneConfig.CurrentLimits.SupplyCurrentLowerTime = 1;
        shootoneConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
        shootoneConfig.Slot0.kV = .11;
        shootoneConfig.Slot0.kP = .34;
        shootoneConfig.Slot0.kS = 0.14;
        shootoneConfig.Slot0.kI = 0;
        shootoneConfig.Slot0.kD = 0;
        shootOne.getConfigurator().apply(shootoneConfig, 0.05);

        TalonFXConfiguration shoottwoConfig = new TalonFXConfiguration();
        shoottwoConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        shoottwoConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        shoottwoConfig.CurrentLimits.SupplyCurrentLimit = 70;
        shoottwoConfig.CurrentLimits.SupplyCurrentLowerTime = 1;
        shoottwoConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
        shoottwoConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shoottwoConfig.Slot0.kV = .11;
        shoottwoConfig.Slot0.kP = .30;
        shoottwoConfig.Slot0.kI = 0;
        shoottwoConfig.Slot0.kS = 0.12;
        shoottwoConfig.Slot0.kD = 0;
        shootTwo.getConfigurator().apply(shoottwoConfig, 0.05);
    }

    public double prevangle=0;
    public double distance = 0;
private boolean inAuto = false;
private boolean justWrapped = false;
double turretWrap = 0;
    @Override
    public void periodic() {
SmartDashboard.putNumber("TurretWrap", turretWrap);
        if((mrotateencoder.getPosition() >=180) && !justWrapped){
           // mrotateencoder.setPosition(mrotateencoder.getPosition()-340);
turretWrap = turretWrap -340;
justWrapped = true;
        } else if ((mrotateencoder.getPosition() <= -180) && !justWrapped){
            //mrotateencoder.setPosition(mrotateencoder.getPosition()+340);
            turretWrap = turretWrap +340;
            justWrapped = true;

        }else if(Math.abs((mrotateencoder.getPosition())) < 180.0){
            justWrapped = false;
            turretWrap = 0;
        }
        double hoodAngle = 0;
        double shooterSpeed = 0;
        Pose3d position = LimelightHelpers.getCameraPose3d_TargetSpace("limelight");
       // autoaimController.setGoal(0);
       // autoaimController.calculate(LimelightHelpers.getTX("limelight"));

        //if Limelight has a valid target,
        if(LimelightHelpers.getTV("limelight")){
            distance = Math.sqrt(position.getZ()*position.getZ()+position.getX()*position.getX());
        }
        
            hoodAngle = hoodmap.get(filter.calculate(distance));
            shooterSpeed = shootmap.get(shootfilter.calculate(distance));
        
        //  double rotateangle = Math.atan2(-position.getZ(), position.getX());
          // double rotateturretleft=turretRotate.
    
        // rotateTurret(autoaimController.calculate(LimelightHelpers.getTX("limelight"))-angledif*.06);
        // prevangle=gyroAngle;

        m_hoodsetpoint = m_pivotProfile.calculate(.02, m_hoodsetpoint, m_hoodGoal);
        m_rotatesetpoint = m_rotateprofile.calculate(.02, m_rotatesetpoint, m_rotategoal);

        SmartDashboard.putNumber("Dist", distance);
        SmartDashboard.putNumber("Hoodangle", hoodAngle);
        SmartDashboard.putNumber("bob2", turretRotate.getEncoder().getPosition());
        //SmartDashboard.putNumber("rotate", rotateangle);

        //Set new position to the PID Controller
     mHoodPID.setReference(m_hoodsetpoint.position, com.revrobotics.spark.SparkBase.ControlType.kPosition); 
    mrotatePID.setReference(m_rotatesetpoint.position, com.revrobotics.spark.SparkBase.ControlType.kPosition); 
 
    // double gyroAngle = 0.0;
    //  gyroAngle = mSwerve.getRotation();
    //  double angledif=gyroAngle-prevangle;
    //        prevangle=gyroAngle;

if(turretInPosition()){
if(LimelightHelpers.getTV("limelight") && !inAuto)
{
   // double limelightOutput = autoaimController.calculate(LimelightHelpers.getTX("limelight"));
        // double anglesign = limelightOutput/Math.abs(limelightOutput); //I know there's a better way to check for the sign, i forget
        // if(Math.abs(limelightOutput)>0.2){
        //     limelightOutput = 0.2*anglesign;
        // }

           rotateTurret((mrotateencoder.getPosition()-LimelightHelpers.getTX("limelight")+turretWrap));//-(angledif*.055));

}else if( LimelightHelpers.getTV("limelight") && inAuto){ //id in auto don't do gyro correction on the turret
    // double limelightOutput = autoaimController.calculate(LimelightHelpers.getTX("limelight"));
        // double anglesign = limelightOutput/Math.abs(limelightOutput); //I know there's a better way to check for the sign, i forget
        // if(Math.abs(limelightOutput)>0.2){
        //     limelightOutput = 0.2*anglesign;
        // }
           rotateTurret((mrotateencoder.getPosition()-LimelightHelpers.getTX("limelight")+turretWrap));
}else{
 //manualrotateTurret(-moperatorController.getLeftX()*.4);//*0.35-(angledif*.05));
 //rotateTurret(mrotateencoder.getPosition());
}
}
    
    // SmartDashboard.putNumber("Gyro Angle:", gyroAngle);
    // SmartDashboard.putNumber("Shooter Speed", getShootOneSpeed());

        if (moperatorController.rightBumper().getAsBoolean() || (inAuto == true)) {
                setHoodAngle(hoodAngle);


            if (getShootOneSpeed() == 0.0 && getShootTwoSpeed() == 0.0) // if the shooters have not been started
            {
                setShootSpeed(shooterSpeed); // start the shooter
            } else if (Math.abs(getShootOneSpeed()) > (shooterSpeed-3.0)) // if the shooters
                                                                                                   // are up to speed
            {
                setSpindexterSpeed(-1.0); // start everything else
                setFeederSpeed(1.0);
                setShootSpeed(shooterSpeed); // start the shooter
            } else {
                // doNothing();
            }
        }else if(moperatorController.povLeft().getAsBoolean() && !LimelightHelpers.getTV("limelight")) {
        
            
        }
        //pass button
        else if (moperatorController.leftBumper().getAsBoolean()) {
        //need to add point to gyro 0 here
        setHoodAngle(3);
        setShootSpeed(70.0);
        if (Math.abs(getShootOneSpeed()) > 65.0) // if the shooters                                                                                  // are up to speed
        {
            setSpindexterSpeed(-1.0); // start everything else
            setFeederSpeed(1.0);
        }

    }
        else{
            setSpindexterSpeed(0);
            setFeederSpeed(0);
            setHoodAngle(0);
            setShootSpeed(0);
            //rotateTurret(0);
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
        SmartDashboard.putNumber("ShooterSpeed", getShootOneSpeed());
        return (shootTwo.getRotorVelocity().getValueAsDouble());
    }

}
