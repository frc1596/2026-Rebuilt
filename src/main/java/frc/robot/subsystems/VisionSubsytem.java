package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsytem extends SubsystemBase{

    private final PhotonCamera camera; 
    private PhotonTrackedTarget bestTarget; 

    public VisionSubsytem()
    {
        // defining photonvision camera
         camera = new PhotonCamera("photon");
    }

    @Override
    public void periodic() {
        PhotonPipelineResult result = camera.getLatestResult();
        if (result.hasTargets()) {
            bestTarget = result.getBestTarget();
        } else {
            bestTarget = null;
        }
        }
    public boolean hasTarget() {
        return bestTarget != null;
    }

    public double getTargetYaw() {
        return hasTarget() ? bestTarget.getYaw() : 0.0;
    }

    public double getDistanceToTarget() {
        return hasTarget() ? bestTarget.getBestCameraToTarget().getX() : 0.0;
    }

    public double getTargetX() {
        return hasTarget() ? bestTarget.getBestCameraToTarget().getX() : 0.0;
    }

    public double getTargetY() {
        return hasTarget() ? bestTarget.getBestCameraToTarget().getY() : 0.0;
    }

    public double getTargetZ() {
        return hasTarget() ? bestTarget.getBestCameraToTarget().getZ() : 0.0;
    }

    public Transform3d getPoseToTarget()
    {
        if (bestTarget != null)
        {
            return bestTarget.getBestCameraToTarget();
        }
        return new Transform3d();
    }
}

