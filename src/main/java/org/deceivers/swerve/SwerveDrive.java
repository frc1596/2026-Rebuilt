package org.deceivers.swerve;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Arrays;
import java.util.function.DoubleSupplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;



public class SwerveDrive {
    //Swerve Devices
    public final SwerveModule[] mModules;
    private final int numModules;
    private final SwerveDriveKinematics mKinematics;
    public final SwerveDrivePoseEstimator mSwerveDrivePoseEstimator;
    private final HolonomicDriveController mDriveController;
    public final DoubleSupplier mGyroAngle;
    private final ProfiledPIDController rotationPIDController = new ProfiledPIDController(15,.1,.1,new TrapezoidProfile.Constraints(500, 500));

    //Network Table Data
    private final NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
    private final DoubleArrayPublisher pathGoalPose;
    private final DoubleArrayPublisher pathGoalPoseError;
    private final DoubleArrayPublisher swervePose;

    public PhotonCamera camera1 = new PhotonCamera("Camera1");
    public PhotonCamera camera2 = new PhotonCamera("Camera2");

    public static final AprilTagFieldLayout kTagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    public static final Transform3d kRobotToCam =
    new Transform3d(new Translation3d( -16.25/39.37,-4.75/39.37, 0.267), new Rotation3d(Math.toRadians(0), Math.toRadians(62),Math.toRadians(180)));
     public static final Transform3d kRobotToCam2 =
    new Transform3d(new Translation3d(-13.5/39.37, -7.75/39.37, 0.267), new Rotation3d(Math.toRadians(0), Math.toRadians(62), Math.toRadians(270)));

    //8.5,2
    public SwerveDrive(DoubleSupplier gyroAngle, SwerveModule... modules){
        mGyroAngle = gyroAngle;
        numModules = modules.length;
        mModules = Arrays.copyOf(modules, numModules);

        //Network Table setup
        NetworkTable swerveTable = networkTableInstance.getTable("swervetable");
        pathGoalPose = swerveTable.getDoubleArrayTopic("/swervetable/pathgoalpose").publish();
        pathGoalPoseError = swerveTable.getDoubleArrayTopic("/swervetable/pathgoalposeerror").publish();
        swervePose = swerveTable.getDoubleArrayTopic("/swervetable/swervepose").publish();

        Translation2d[] moduleLocations = new Translation2d[numModules];
        for (int i = 0; i < numModules; i++){
            moduleLocations[i] = mModules[i].getModuleLocation();
        }

        mKinematics = new SwerveDriveKinematics(moduleLocations);

        rotationPIDController.enableContinuousInput(-Math.PI, Math.PI);

        mDriveController = new HolonomicDriveController(new PIDController(4,0,0), new PIDController(4,0,0), rotationPIDController);

        SwerveModulePosition[] states = new SwerveModulePosition[numModules];
        for (int i = 0; i < numModules; i++) {
            states[i] = mModules[i].getPosition();
        }

        mSwerveDrivePoseEstimator = new SwerveDrivePoseEstimator(mKinematics, Rotation2d.fromDegrees(mGyroAngle.getAsDouble()), states, new Pose2d(), VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(360)));

        Arrays.stream(mModules).forEach(SwerveModule::init);
    }

    public void periodic(){
        SwerveModulePosition[] states = new SwerveModulePosition[numModules];
        for (int i = 0; i < numModules; i++) {
            states[i] = mModules[i].getPosition();
        }
//photonEstimator.setRobotToCameraTransform(kRobotToCam);
        mSwerveDrivePoseEstimator.update(Rotation2d.fromDegrees(mGyroAngle.getAsDouble()), states);

        var result1 = camera1.getLatestResult();
        var target = result1.getBestTarget();

        // Calculate robot's field relative pose
        if(target != null){
            if (kTagLayout.getTagPose(target.getFiducialId()).isPresent()) {

                Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), kTagLayout.getTagPose(target.getFiducialId()).get(), kRobotToCam);
                Pose2d robot2dpose = robotPose.toPose2d();
                 mSwerveDrivePoseEstimator.addVisionMeasurement(robot2dpose, result1.getTimestampSeconds());
            }
        }

        var result2 = camera2.getLatestResult();
        var target2 = result2.getBestTarget();
        if(target2 != null){
        // Calculate robot's field relative pose
            if (kTagLayout.getTagPose(target2.getFiducialId()).isPresent()) {
                Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target2.getBestCameraToTarget(), kTagLayout.getTagPose(target2.getFiducialId()).get(), kRobotToCam);
                Pose2d robot2dpose = robotPose.toPose2d();
                mSwerveDrivePoseEstimator.addVisionMeasurement(robot2dpose, result2.getTimestampSeconds());

            }
        }

        SmartDashboard.putNumber("GyroAngle", mGyroAngle.getAsDouble());

    }

    public void setLocation(double x, double y, double angle){
        Pose2d newPose = new Pose2d(x, y, Rotation2d.fromDegrees(angle));
        SwerveModulePosition[] states = new SwerveModulePosition[numModules];
        for (int i = 0; i < numModules; i++) {
            states[i] = mModules[i].getPosition();
        }

        mSwerveDrivePoseEstimator.resetPosition(Rotation2d.fromDegrees(mGyroAngle.getAsDouble()), states, newPose); // maybe add a modulus here
        rotationPIDController.reset(angle);
    }

    public void setModulesAngle(double angle, int module){
        mModules[module].setAngle(angle);
    }

    public Pose2d getPose(){
        return mSwerveDrivePoseEstimator.getEstimatedPosition();
    }

    public void updatePoseWithVision(Pose2d newPose, double time){
        mSwerveDrivePoseEstimator.addVisionMeasurement(newPose, time);
    }

    public ChassisSpeeds getChassisSpeeds(){
        SwerveModuleState[] states = new SwerveModuleState[numModules];
        for (int i = 0; i < numModules; i++) {
            states[i] = mModules[i].getState();
        }
        ChassisSpeeds chassisSpeeds = mKinematics.toChassisSpeeds(states);
        return(chassisSpeeds);
    }

	public void stop() {
        Arrays.stream(mModules).forEach(SwerveModule::stop);
	}

    public void drive(double forward, double strafe, double azimuth, boolean fieldRelative){
        ChassisSpeeds speeds;
        if (fieldRelative){
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, azimuth, Rotation2d.fromDegrees(mGyroAngle.getAsDouble()));
        } else {
            speeds = new ChassisSpeeds(forward, strafe, azimuth);
        }
        SwerveModuleState[] states = mKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, 1);
        for (int i = 0; i < numModules; i++){
            mModules[i].set(states[i]);
        }
    }

    public void drivePathplanner(ChassisSpeeds wowChassisSpeeds){
        //invert the rotations if it turns backwards
        ChassisSpeeds wow2ChassisSpeeds = new ChassisSpeeds(wowChassisSpeeds.vxMetersPerSecond,wowChassisSpeeds.vyMetersPerSecond,wowChassisSpeeds.omegaRadiansPerSecond);
        SwerveModuleState[] states = mKinematics.toSwerveModuleStates(wow2ChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, 1);
        for (int i = 0; i < numModules; i++){
            mModules[i].set(states[i]);
        }
    }

    public void resetPosePathplanner(Pose2d spin){
        SwerveModulePosition[] states = new SwerveModulePosition[numModules];
        for (int i = 0; i < numModules; i++) {
            states[i] = mModules[i].getPosition();
        }
        mSwerveDrivePoseEstimator.resetPosition(Rotation2d.fromDegrees(mGyroAngle.getAsDouble()), states, spin);
    }
    
    public void driveClosedLoop(ChassisSpeeds speeds){
        // if (fieldRelative){
        //     speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, azimuth, Rotation2d.fromDegrees(mGyroAngle.getAsDouble()));
        // } else {
        //     speeds = new ChassisSpeeds(forward, strafe, azimuth);
        // }
        SwerveModuleState[] states = mKinematics.toSwerveModuleStates(speeds);
        //SwerveDriveKinematics.normalizeWheelSpeeds(states, 1);
        for (int i = 0; i < numModules; i++){
            mModules[i].setClosedLoop(states[i]);
        }
    }

    public Pose2d updateOdometry(){
        SwerveModulePosition[] states = new SwerveModulePosition[numModules];
        for (int i = 0; i < numModules; i++) {
            states[i] = mModules[i].getPosition();
        }

        return mSwerveDrivePoseEstimator.update(Rotation2d.fromDegrees(mGyroAngle.getAsDouble()), states);
    }

    public void log(){
        Arrays.stream(mModules).forEach(SwerveModule::log);
        double[] swervePoseArray = {getPose().getX(), getPose().getY(), getPose().getRotation().getDegrees()};
        swervePose.set(swervePoseArray);

        // if (LimelightHelpers.getTV("")){
        // Pose2d botPose = LimelightHelpers.getBotPose2d_wpiBlue("");
        // double timeDelay = Timer.getFPGATimestamp() - (LimelightHelpers.getLatency_Capture("")/1000.0) - (LimelightHelpers.getLatency_Pipeline("")/1000.0);

        // SmartDashboard.putNumber("limelightTIme", timeDelay);
        // SmartDashboard.putNumber("limex", botPose.getY());
        // SmartDashboard.putNumber("limey", botPose.getX());
        // SmartDashboard.putNumber("limeRot", botPose.getRotation().getDegrees());
        // SmartDashboard.putNumber("odometryRotation", mSwerveDrivePoseEstimator.getEstimatedPosition().getRotation().getDegrees());
        // SmartDashboard.putNumber("Tags", LimelightHelpers.getLatestResults("").targetingResults.targets_Retro.length);
        //}
    }
}