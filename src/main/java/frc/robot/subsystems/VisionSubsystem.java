// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  private AprilTagFieldLayout apriltaglayout;
  boolean targetVisible = false;
  private PhotonCamera camera = new PhotonCamera("cam");
  private DriveTrain m_swerve;

  List<PhotonPipelineResult> results = camera.getAllUnreadResults();
  public VisionSubsystem() {
    apriltaglayout = VisionConstants.APRIL_TAGS_LAYOUT;

  }

  @Override
  public void periodic() {
    results = camera.getAllUnreadResults();
    if (!results.isEmpty()) {
      PhotonPipelineResult result = results.get(results.size() - 1);
      if (result.hasTargets()) {
        for (var target : result.getTargets()) {
          // im giving up
        }
      }
    }
    // This method will be called once per scheduler run
  }
}
