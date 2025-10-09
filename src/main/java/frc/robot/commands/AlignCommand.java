// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PoseEstimatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignCommand extends Command {
  /** Creates a new AlignCommand. */
  private final DriveTrain m_swerve;
  private final PoseEstimatorSubsystem m_vision;
  private final boolean targetRightCoral;
  private static final TrapezoidProfile.Constraints xConstraints = new TrapezoidProfile.Constraints(2, 3);
  private static final TrapezoidProfile.Constraints yConstraints = new TrapezoidProfile.Constraints(2, 3);
  private static final TrapezoidProfile.Constraints rotConstraints = new TrapezoidProfile.Constraints(2 * Math.PI, 2 * Math.PI);

  private final ProfiledPIDController xController = new ProfiledPIDController(AutoConstants.translationkP, AutoConstants.translationkI, AutoConstants.translationkD, xConstraints);
  private final ProfiledPIDController yController = new ProfiledPIDController(AutoConstants.translationkP, AutoConstants.translationkI, AutoConstants.translationkD, yConstraints);
  private final ProfiledPIDController rotController = new ProfiledPIDController(AutoConstants.thetakP, AutoConstants.thetakI, AutoConstants.thetakD, rotConstraints);


  private Pose2d targetTagPose; // transform3d
  private Transform3d targetTagTransform3d;
  private Pose2d targetPose;
  private Pose2d robotPose;
  private Pose2d goalPoseV2;
  public AlignCommand(DriveTrain m_swerve, PoseEstimatorSubsystem m_vision, boolean targetRightCoral) {
    this.m_swerve = m_swerve;
    this.m_vision = m_vision;
    this.targetRightCoral = targetRightCoral;
    rotController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(m_swerve, m_vision);
  }

  @Override
  public void initialize() {
    xController.setTolerance(0.05, 0.1);
    yController.setTolerance(0.05, 0.1);
    rotController.setTolerance(Units.degreesToRadians(3));

    // could use this but eh
    // robotPose = m_vision.getPoseEstimate();

    robotPose = m_swerve.getPose();
    rotController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
    // targetTagPose = m_vision.findBestTagPose().toPose2d();
    targetTagTransform3d = m_vision.findCameraRelativeTagPose();
    calculateTargetPose();
  }

  private void calculateTargetPose() {
    // make a translation for offset and add the pose of target stuf
    // THIS STUFF IS BACK UP
    
    var robotPose3d = new Pose3d(robotPose.getX(), robotPose.getY(), 0,
      new Rotation3d(0, 0, robotPose.getRotation().getRadians()));
    var cameraPose = robotPose3d.transformBy(VisionConstants.camPosition);
    var camToTarget = targetTagTransform3d;
    // the tag pose relative to camera
    var targetPoseV2 = cameraPose.transformBy(camToTarget).toPose2d();
    // UP TO HERE
    SmartDashboard.putString("Target Pose", String.format("(%.2f, %.2f) %.2f degrees", 
          targetPoseV2.getX(),
          targetPoseV2.getY(), 
          targetPoseV2.getRotation().getDegrees()));


    double lateralOffset = targetRightCoral ? AutoConstants.coralRightOffset : AutoConstants.coralLeftOffset;
    Translation2d lateralOffsetTranslation = new Translation2d(0, lateralOffset);
    lateralOffsetTranslation = lateralOffsetTranslation.rotateBy(targetPoseV2.getRotation());

    Translation2d approachOffset = new Translation2d(AutoConstants.robotCenterToFrontDistance, 0);
    // approachOffset = approachOffset.rotateBy(targetTagPose.getRotation());
    approachOffset = approachOffset.rotateBy(targetPoseV2.getRotation());

    // targetPose = new Pose2d(
    //   targetTagPose.getX() + lateralOffsetTranslation.getX() + approachOffset.getX(),
    //   targetTagPose.getY() + lateralOffsetTranslation.getY() + approachOffset.getY(),
    //   targetTagPose.getRotation().rotateBy(Rotation2d.k180deg));


    goalPoseV2 = new Pose2d(
      targetPoseV2.getX() + lateralOffsetTranslation.getX() + approachOffset.getX(),
      targetPoseV2.getY() + lateralOffsetTranslation.getY() + approachOffset.getY(),
      targetPoseV2.getRotation().rotateBy(Rotation2d.k180deg));
    SmartDashboard.putString("Goal Pose", String.format("(%.2f, %.2f) %.2f degrees", 
          goalPoseV2.getX(),
          goalPoseV2.getY(), 
          goalPoseV2.getRotation().getDegrees()));
  }

  
  @Override
  public void execute() {
    // var currentPose2d = m_vision.getPoseEstimate();
    // var currentPose = 
    //     new Pose3d(
    //       currentPose2d.getX(),
    //       currentPose2d.getY(),
    //       0,
    //       new Rotation3d(0,0, currentPose2d.getRotation().getRadians())
    //     );
    // var cameraPose = currentPose.transformBy(VisionConstants.camPosition);
    // var camToTarget = m_vision.findCameraRelativeTagPose();
    // var targetPose = cameraPose.transformBy(camToTarget);
    
   
    Pose2d currentPose = m_swerve.getPose();
    SmartDashboard.putString("Current Pose", String.format("(%.2f, %.2f) %.2f degrees", 
          currentPose.getX(),
          currentPose.getY(), 
          currentPose.getRotation().getDegrees()));
    // Pose2d currentPose = m_vision.getPoseEstimate();

    double xSpeed = xController.calculate(currentPose.getX(), goalPoseV2.getX());
    double ySpeed = yController.calculate(currentPose.getY(), goalPoseV2.getY());
    double rot = rotController.calculate(currentPose.getRotation().getRadians(), goalPoseV2.getRotation().getRadians());

    xSpeed = MathUtil.clamp(xSpeed, -AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxSpeedMetersPerSecond);
    ySpeed = MathUtil.clamp(ySpeed, -AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxSpeedMetersPerSecond);
    rot = MathUtil.clamp(rot, -AutoConstants.kMaxAngularSpeedRadiansPerSecond, AutoConstants.kMaxAngularSpeedRadiansPerSecond);

    m_swerve.drive(xSpeed, ySpeed, rot, true);
  }


 
  @Override
  public void end(boolean interrupted) {
    m_swerve.drive(0,0,0,true);
  }

  
  @Override
  public boolean isFinished() {
    return xController.atGoal() && yController.atGoal() && rotController.atGoal();
  }
}
