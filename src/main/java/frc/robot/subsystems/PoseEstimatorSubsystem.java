// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;

public class PoseEstimatorSubsystem extends SubsystemBase {
  // https://github.com/STMARobotics/frc-7028-2023/blob/5916bb426b97f10e17d9dfd5ec6c3b6fda49a7ce/src/main/java/frc/robot/subsystems/PoseEstimatorSubsystem.java#L11
  private AprilTagFieldLayout apriltaglayout; // layout of field tags

  private PhotonCamera camera = new PhotonCamera("photonvision"); // create camera
  private DriveTrain m_swerve;
  private PhotonTrackedTarget currentTarget;

  private boolean hasTarget = false;
  private double previousPipelineTimestamp = 0;
  // Initalize list of tagPoses to access with IDs
  private List<Pose3d> tagPoses = new ArrayList<>();

  private static final edu.wpi.first.math.Vector<N3> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1); // 0.1, 0.1, 0.1
  private static final edu.wpi.first.math.Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);

  
  private final SwerveDrivePoseEstimator poseEstimator; // should be SwerveDrivePoseEstimator<N7, N7, N5>
  private final Field2d field2d = new Field2d();
  public PoseEstimatorSubsystem(DriveTrain m_swerve) {
    apriltaglayout = VisionConstants.APRIL_TAGS_LAYOUT;
    this.m_swerve = m_swerve;
    poseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      m_swerve.getGyroRotation(),
      m_swerve.getSwerveModulePositions(), // m_swerve.getDrivetrainState().getSwerveModulePositions(),
      new Pose2d(), 
      stateStdDevs,
      visionMeasurementStdDevs
    );
    
    for (int tagId : VisionConstants.tagIds) {
      var tagPoseOptional = apriltaglayout.getTagPose(tagId);
      if (tagPoseOptional.isPresent()) {
        tagPoses.add(tagPoseOptional.get());// .toPose2d()
      }
    }
  }
  public boolean hasTarget() {
    return hasTarget;
  }
  public Pose2d getPoseEstimate() {
    return poseEstimator.getEstimatedPosition();
  }

  public Pose3d getCurrentTargetPose() {
    return tagPoses.get(currentTarget.getFiducialId() - 1);
  }

  public Transform3d findCameraRelativeTagPose() {
    return currentTarget.getBestCameraToTarget();
  }

  public PhotonPipelineResult getNewestResult() {
    return camera.getLatestResult();
  }

  /**
   * 
   * @param approachOffset The offset distance in front of the tag to be (positive being further from tag)
   * @param lateralOffset The offset distance to the side of the tag (positive being right, negative being left)
   * @param rotationalOffset
   * @return Returns a robot pose with the given offsets relative to tag (including bumpers)
   */
  public Pose2d calculateGoalPoseForBestTag(double approachOffset, double lateralOffset, Rotation2d rotationalOffset) {
    Pose2d robotPose = m_swerve.getPose();
    Transform3d targetTransformation = currentTarget.getBestCameraToTarget();
    var robotPose3d = new Pose3d(robotPose.getX(), robotPose.getY(), 0,
      new Rotation3d(0, 0, robotPose.getRotation().getRadians()));
    var cameraPose = robotPose3d.transformBy(VisionConstants.camPosition);
    var camToTarget = targetTransformation;
    var targetPose = cameraPose.transformBy(camToTarget).toPose2d();
    Translation2d lateralOffsetTranslation = new Translation2d(0, lateralOffset).rotateBy(camToTarget.getRotation().toRotation2d());
    Translation2d approachOffsetTranslation = new Translation2d(AutoConstants.robotCenterToFrontDistance + approachOffset, 0).rotateBy(camToTarget.getRotation().toRotation2d());

    Pose2d goalPose = new Pose2d(
      targetPose.getX() + lateralOffsetTranslation.getX() + approachOffsetTranslation.getX(),
      targetPose.getY() + lateralOffsetTranslation.getY() + approachOffsetTranslation.getY(),
      targetPose.getRotation().rotateBy(Rotation2d.k180deg.plus(rotationalOffset)));
    return goalPose;
  }


  @Override
  public void periodic() {
    SmartDashboard.putBoolean("cam connected", camera.isConnected());
    var pipelineResult = camera.getLatestResult();
    var resultTimestamp = pipelineResult.getTimestampSeconds();
    SmartDashboard.putBoolean("cam has targets", pipelineResult.hasTargets());
    if (resultTimestamp != previousPipelineTimestamp && pipelineResult.hasTargets()) {
      previousPipelineTimestamp = resultTimestamp;
      hasTarget = true;
      var target = pipelineResult.getBestTarget();
      currentTarget = target;
      int fiducialId = target.getFiducialId();
      if (target.getPoseAmbiguity() <= 0.2 && fiducialId >= 0 && fiducialId < tagPoses.size()) {
        var targetPose = tagPoses.get(fiducialId - 1);
        Transform3d camToTarget = target.getBestCameraToTarget();
        Pose3d camPose = targetPose.transformBy(camToTarget.inverse());

        var visionMeasurement = camPose.transformBy(VisionConstants.camPosition);
        poseEstimator.addVisionMeasurement(visionMeasurement.toPose2d(), resultTimestamp);
      }
    }
    // Update pose estimator
    var drivetrainState = m_swerve.getPose();
    poseEstimator.update(
      m_swerve.getGyroRotation(), // also a m_swerve.getSwerveModuleStates() but thats too many args?
      m_swerve.getSwerveModulePositions());
    field2d.setRobotPose(poseEstimator.getEstimatedPosition()); // m_swerve.getPose() maybe??
    SmartDashboard.putData(field2d);
  }
}
