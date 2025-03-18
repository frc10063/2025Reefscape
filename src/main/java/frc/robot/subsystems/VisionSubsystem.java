// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import java.util.ArrayList;
// import java.util.List;
// import java.util.Vector;

// import org.photonvision.PhotonCamera;
// import org.photonvision.targeting.PhotonPipelineResult;
// import org.photonvision.targeting.PhotonTrackedTarget;

// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.math.Nat;
// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.numbers.N3;
// import edu.wpi.first.math.numbers.N5;
// import edu.wpi.first.math.numbers.N7;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.DriveConstants;
// import frc.robot.Constants.VisionConstants;

// public class VisionSubsystem extends SubsystemBase {
  
//   private AprilTagFieldLayout apriltaglayout; // layout of field tags

//   private PhotonCamera camera = new PhotonCamera("cam"); // create camera
//   private DriveTrain m_swerve;
//   private PhotonTrackedTarget currentTarget;

//   private boolean hasTarget = false;
//   private double previousPipelineTimestamp = 0;
//   // Initalize list of tagPoses to access with IDs
//   private List<Pose3d> tagPoses = new ArrayList<>();

//   // it tells me to import numbers and hcange to N1 or N3 when they all in smae import?
//   // this is benson work idk how to do statistics and determine values
//   private static final edu.wpi.first.math.Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
//   // private static final edu.wpi.first.math.Vector<N3> localMeasurementStdDevs = VecBuilder.fill(Units.degreesToRadians(0.01), 0.01, 0);
//   private static final edu.wpi.first.math.Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

  
//   private final SwerveDrivePoseEstimator poseEstimator; // should be SwerveDrivePoseEstimator<N7, N7, N5>
//   // Field2d can be put on smartdashboard
//   private final Field2d field2d = new Field2d();
//   public VisionSubsystem() {
//     apriltaglayout = VisionConstants.APRIL_TAGS_LAYOUT;
//     m_swerve = new DriveTrain();
//     // this is my sign to be a liberal arts major
//     poseEstimator = new SwerveDrivePoseEstimator(
//       DriveConstants.kDriveKinematics,
//       m_swerve.getGyroRotation(),
//       m_swerve.getSwerveModulePositions(), // m_swerve.getDrivetrainState().getSwerveModulePositions(),
//       new Pose2d(),
//       stateStdDevs,
//       visionMeasurementStdDevs
//     );
//     // loop to add tag poses to list
//     for (int tagId : VisionConstants.tagIds) {
//       var tagPoseOptional = apriltaglayout.getTagPose(tagId);
//       if (tagPoseOptional.isPresent()) {
//         // do i want 2d or 3d poses?
//         tagPoses.add(tagPoseOptional.get());// .toPose2d()
//       }
//     }
//   }
//   public boolean hasTarget() {
//     return hasTarget;
//   }
//   public Pose2d getPoseEstimate() {
//     return poseEstimator.getEstimatedPosition();
//   }

//   public Pose3d findBestTagPose() {
//     return tagPoses.get(currentTarget.getFiducialId());
//   }
//   public PhotonPipelineResult getNewestResult() {
//     return camera.getLatestResult();
//   }


//   @Override
//   public void periodic() {
//     // this is deprecated, would need to make array and get last item
//     var pipelineResult = camera.getLatestResult();
//     var resultTimestamp = pipelineResult.getTimestampSeconds();

//     if (resultTimestamp != previousPipelineTimestamp && pipelineResult.hasTargets()) {
//       previousPipelineTimestamp = resultTimestamp;
//       hasTarget = true;
//       // somethings prob deprecated
//       var target = pipelineResult.getBestTarget();
//       currentTarget = target;
//       // var?
//       int fiducalId = target.getFiducialId();
//       if (target.getPoseAmbiguity() <= 0.2 && fiducalId >= 0 && fiducalId < tagPoses.size()) {
//         var targetPose = tagPoses.get(fiducalId);
//         Transform3d camToTarget = target.getBestCameraToTarget();
//         Pose3d camPose = targetPose.transformBy(camToTarget.inverse());

//         var visionMeasurement = camPose.transformBy(VisionConstants.camPosition);
//         poseEstimator.addVisionMeasurement(visionMeasurement.toPose2d(), resultTimestamp);
//       }
//     }
//     // Update pose estimator
//     var drivetrainState = m_swerve.getPose();
//     poseEstimator.update(
//       m_swerve.getGyroRotation(), // also a m_swerve.getSwerveModuleStates() but thats too many args?
//       m_swerve.getSwerveModulePositions());
//     // why is this stuff only error for me what is this method supposed to be called on 
//     field2d.setRobotPose(poseEstimator.getEstimatedPosition()); // m_swerve.getPose() maybe??
//     SmartDashboard.putData(field2d);
//   }
// }
