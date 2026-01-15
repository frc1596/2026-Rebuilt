package frc.robot.subsystems;

import org.deceivers.drivers.LimelightHelpers;
import org.deceivers.drivers.LimelightHelpers.LimelightResults;
import org.deceivers.drivers.LimelightHelpers.LimelightTarget_Fiducial;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  public Limelight() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean getTV(){
    return LimelightHelpers.getTV("");
  }
  
  public Pose2d getPose(){
    return LimelightHelpers.getBotPose2d_wpiBlue("");
  }
  
public double getFid(){
  return LimelightHelpers.getFiducialID("");
}
  public double getTimeStamp(){
    return Timer.getFPGATimestamp() - (LimelightHelpers.getLatency_Pipeline("")/1000.0)+(LimelightHelpers.getLatency_Capture("")/1000.0);
  }
}