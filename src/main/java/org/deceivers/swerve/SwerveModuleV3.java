package org.deceivers.swerve;

import com.revrobotics.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;


import com.ctre.phoenix6.configs.CANcoderConfiguration;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModuleV3 implements SwerveModule {

    private final SparkMax mAzimuthMotor;
    private final SparkMax mDriveMotor;


    private final CANcoder mAzimuthAbsoluteEncoder;


    private final RelativeEncoder mAzimuthIncrementalEncoder;
    private final RelativeEncoder mDriveEncoder;

    private final Translation2d mLocation;
    private final String mName;

      private PIDController azimuthPID = new PIDController(0.01, 0, 0);

    // need to update the speed to m/s

    public SwerveModuleV3(SparkMax azimuthMotor, SparkMax driveMotor,
            Translation2d location, String name, CANcoder azimuthEncoder) {
            
        mDriveMotor = driveMotor;
        mAzimuthMotor = azimuthMotor;
        mLocation = location;
        mName = name;

        // Rest motors to factory defaults to ensure correct parameters
        //mDriveMotor.restoreFactoryDefaults();
        //mAzimuthMotor.restoreFactoryDefaults();

        // Get encoders
        mAzimuthAbsoluteEncoder = azimuthEncoder;
        mAzimuthIncrementalEncoder = mAzimuthMotor.getEncoder();
        mDriveEncoder = mDriveMotor.getEncoder();


        SparkMaxConfig driveConfig = new SparkMaxConfig();

        driveConfig
    .inverted(true)
    .idleMode(IdleMode.kBrake);
    driveConfig.encoder
    .positionConversionFactor(0.319024/6.12)
    .velocityConversionFactor(((0.319024/6.12) / 60));
    driveConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(1.0, 0.0, 0.0)
    .velocityFF(0.0);
    
    
mDriveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

SparkMaxConfig azimuthConfig = new SparkMaxConfig();

azimuthConfig
.inverted(true)
.idleMode(IdleMode.kBrake);
azimuthConfig.encoder
.positionConversionFactor(150.0/7.0)
.velocityConversionFactor(1);
azimuthConfig.closedLoop
.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
.pid(0.05, 0.0, 0.0)
.positionWrappingMaxInput(360)
.positionWrappingMinInput(0);


mDriveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);



        azimuthPID.enableContinuousInput(-180,180);
        // Configure azimuth PID
        //mAzimuthPID.setFeedbackDevice(mAzimuthAbsoluteEncoder);
        // mAzimuthPID.setFeedbackDevice(mAzimuthIncrementalEncoder);
        // mAzimuthPID.setP(.05);
        // mAzimuthPID.setPositionPIDWrappingEnabled(true);
        // mAzimuthPID.setPositionPIDWrappingMinInput(0);
        // mAzimuthPID.setPositionPIDWrappingMaxInput(360);



    }

    // Sets the drive motor speed in open loop mode
    public void setSpeed(double speed) {
        mDriveMotor.set(speed);
    }

    // Sets the rotation speed of the azimuth motor in open loop mode
    public void setRotation(double rotation) {
        mAzimuthMotor.set(rotation);
    }

    // Gets the speed of the drive motor
    public double getSpeed() {
        return mDriveEncoder.getVelocity();
    }

    // Gets the rotation position of the azimuth module
    public double getRotation() {
        return mAzimuthAbsoluteEncoder.getPosition().getValueAsDouble()*360.0;
    }

    // Gets the x/y location of the module relative to the center of the robot
    @Override
    public Translation2d getModuleLocation() {
        return mLocation;
    }

    // Get the state (speed/rotation) of the swerve module
    public SwerveModuleState getState() {
        return new SwerveModuleState(getSpeed(), Rotation2d.fromDegrees(getRotation()));
    }

    // Run when swerve drive is first initialized
    @Override
    public void init() {

    }

    // Get the position of swerve modules (distance and angle)
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDistance(), Rotation2d.fromDegrees(getRotation()));
    }

    // Get the distance of the drive encoder
    public double getDistance() {
        return mDriveEncoder.getPosition();
    }

    // Log swerve data
    @Override
    public void log() {
        //SmartDashboard.putNumber(mName + " Current", mDriveMotor.getOutputCurrent());
        //SmartDashboard.putNumber(mName + " Speed", mDriveEncoder.getVelocity());
    }

    // Set the speed and direction of the swerve module
    @Override
    public void set(SwerveModuleState drive) {
        Rotation2d current = Rotation2d.fromDegrees(mAzimuthAbsoluteEncoder.getPosition().getValueAsDouble()*360.0);
        SwerveModuleState optimizedState = SwerveModuleState.optimize(drive, current);
        double setpoint = optimizedState.angle.getDegrees();
        double velocity = optimizedState.speedMetersPerSecond;
        //mAzimuthPID.setReference(setpoint, ControlType.kPosition);
        mAzimuthMotor.set(azimuthPID.calculate(current.getDegrees(), setpoint));
        mDriveMotor.set(velocity);
        //SmartDashboard.putNumber(mName + "Angle", current.getDegrees());
    }

    @Override
    public void setClosedLoop(SwerveModuleState drive) {
        Rotation2d current = Rotation2d.fromDegrees(mAzimuthAbsoluteEncoder.getPosition().getValueAsDouble()*360.0);
        SwerveModuleState optimizedState = SwerveModuleState.optimize(drive, current);
        double setpoint = optimizedState.angle.getDegrees();
        double velocity = optimizedState.speedMetersPerSecond;
        //mAzimuthPID.setReference(setpoint, ControlType.kPosition);
        
        mAzimuthMotor.set(azimuthPID.calculate(current.getDegrees(), setpoint));
        //mDrivePID.setReference(velocity, ControlType.kVelocity);
    }

    // Stop all motors
    @Override
    public void stop() {
        mAzimuthMotor.set(0);
        mDriveMotor.set(0);
    }

    // set angle of swerve drive
    @Override
    public void setAngle(double angle) {
       // mAzimuthPID.setReference(angle, ControlType.kPosition);
        azimuthPID.calculate(mAzimuthAbsoluteEncoder.getPosition().getValueAsDouble()*360.0, angle);

    }

}
