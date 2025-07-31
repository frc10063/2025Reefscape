// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonUtils;

// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;

// public class VisionSubsystem extends SubsystemBase {
//   /** Creates a new VisionSubsystem. */
//   private PhotonCamera camera = new PhotonCamera("photonvision"); // create camera
//   private DriveTrain m_swerve;
//   public VisionSubsystem() {
//   }

//   @Override
//   public void periodic() {
//     double forward = 0;
//     double strafe = 0;
//     double turn = 0;
//     // Read in relevant data from the Camera
//     boolean targetVisible = false;
//     double targetYaw = 0.0;
//     double targetRange = 0.0;
//     var results = camera.getAllUnreadResults();
//     if (!results.isEmpty()) {
//       // Camera processed a new frame since last
//       // Get the last one in the list.
//       var result = results.get(results.size() - 1);
//       if (result.hasTargets()) {
//         // At least one AprilTag was seen by the camera
//         for (var target : result.getTargets()) {
//           if (target.getFiducialId() == 7) {
//             // Found Tag 7, record its information
//             targetYaw = target.getYaw();
//             targetRange = PhotonUtils.calculateDistanceToTargetMeters(
//                 0.5, // Measured with a tape measure, or in CAD.
//                 1.435, // From 2024 game manual for ID 7
//                 Units.degreesToRadians(-30.0), // Measured with a protractor, or in CAD.
//                 Units.degreesToRadians(target.getPitch()));

//             targetVisible = true;
//           }
//         }
//       }
//     }

//   turn = (VISION_DES_ANGLE_deg - targetYaw) * VISION_TURN_kP * Constants.Swerve.kMaxAngularSpeed;
//   forward = (VISION_DES_RANGE_m - targetRange) * VISION_STRAFE_kP * Constants.Swerve.kMaxLinearSpeed;

//     drivetrain.drive(forward, strafe, turn);
//   }
// }
