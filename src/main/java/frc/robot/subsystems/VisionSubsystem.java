// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  private AprilTagFieldLayout apriltaglayout;
  private PhotonCamera camera = new PhotonCamera("cam");
  private DriveTrain m_swerve;
  private double previousPipelineTimestamp = 0;
  private List<Pose2d> tagPoses = new ArrayList<>();

  public VisionSubsystem() {
    apriltaglayout = VisionConstants.APRIL_TAGS_LAYOUT;
    for (int tagId : VisionConstants.tagIds) {
      var tagPoseOptional = apriltaglayout.getTagPose(tagId);
      if (tagPoseOptional.isPresent()) {
        tagPoses.add(tagPoseOptional.get().toPose2d());
      }
    }
  }

  @Override
  public void periodic() {
    // this is deprecated, would need to make array and get last item
    var pipelineResult = camera.getLatestResult();
    var resultTimestamp = pipelineResult.getTimestampSeconds();

    if (resultTimestamp != previousPipelineTimestamp && pipelineResult.hasTargets()) {
      previousPipelineTimestamp = resultTimestamp;
      // somethings prob deprecated
      var target = pipelineResult.getBestTarget();
      // var?
      int fiducalId = target.getFiducialId();
      // if (target.getPoseAmbiguity() <= 0.2 && fiducalId >= 0 && fiducalId < tagPoses.size()) {
      //   var targetPose = tagPoses.get(fiducalId);
      //   Transform3d camToTarget = target.getBestCameraToTarget();
      //   Pose3d camPose = targetPose.transformBy(camToTarget.inverse());

      //   var visionMeasure = camPose.transformBy(VisionConstants.camPosition);
      // }
    }
    // This method will be called once per scheduler run
  }
}
