// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PoseEstimatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ChaseTagCommand extends Command {
  /** Creates a new ChaseTagCommand. */
  // this is honestly useless but it cool to mess around with
  private static final TrapezoidProfile.Constraints xConstraints = new TrapezoidProfile.Constraints(0.5, 0.5);
  private static final TrapezoidProfile.Constraints yConstraints = new TrapezoidProfile.Constraints(0.5, 0.5);
  private static final TrapezoidProfile.Constraints rotConstraints = new TrapezoidProfile.Constraints(Math.PI, Math.PI);

  private static final int tagToChase = 15;
  private static final Transform3d tagToGoal = 
      new Transform3d(1, 0, 0,
          new Rotation3d(0, 0, Math.PI)); // yaw may be -pi/2 if front hasnt changed
//   private final PhotonCamera photonCamera;
  private final DriveTrain swerve;
  private final PoseEstimatorSubsystem vision;

  private final ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, xConstraints);
  private final ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, yConstraints);
  private final ProfiledPIDController rotController = new ProfiledPIDController(3, 0, 0, rotConstraints);

  private PhotonTrackedTarget lastTarget;

  public ChaseTagCommand(PoseEstimatorSubsystem vision, DriveTrain swerve) {
    this.swerve = swerve;
    this.vision = vision;

    xController.setTolerance(0.1);
    yController.setTolerance(0.1);
    rotController.setTolerance(Units.degreesToRadians(5));
    rotController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastTarget = null;
    var robotPose = swerve.getPose();
    rotController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var robotPose2d = swerve.getPose();
    var robotPose = 
        new Pose3d(
          robotPose2d.getX(),
          robotPose2d.getY(),
          0,
          new Rotation3d(0,0, robotPose2d.getRotation().getRadians())
        );
    SmartDashboard.putString("Current Pose", String.format("(%.2f, %.2f) %.2f degrees", 
      robotPose2d.getX(),
      robotPose2d.getY(), 
      robotPose2d.getRotation().getDegrees()));
    var photonRes = vision.getNewestResult();
    if (photonRes.hasTargets()) {
      var targetOpt = photonRes.getTargets().stream()
          .filter(t -> t.getFiducialId() == tagToChase)
          .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= 0.2 && t.getPoseAmbiguity() != -1) 
          .findFirst();
      if (targetOpt.isPresent()) {
        var target = targetOpt.get();
        lastTarget = target;
        // ima try to simplify this
        // the camera pose is set to the estimated robot pose 
        // accounting for where the camera is on the bot
        var cameraPose = robotPose.transformBy(VisionConstants.camPosition);
        // camToTarget represents the transformation needed to get from camera to target
        var camToTarget = target.getBestCameraToTarget();
        SmartDashboard.putString("Best Cam To Target", String.format("(%.2f, %.2f, %.2f) rotation: (%.2f, %.2f, %.2f)", 
          camToTarget.getX(),
          camToTarget.getY(),
          camToTarget.getZ(),
          camToTarget.getRotation().getX(),
          camToTarget.getRotation().getY(),
          camToTarget.getRotation().getZ()));
        // the target pose is the distance of camera position to the target
        var targetPose = cameraPose.transformBy(camToTarget);
        SmartDashboard.putString("Target Pose", String.format("(%.2f, %.2f) %.2f degrees", 
          targetPose.getX(),
          targetPose.getY(), 
          targetPose.toPose2d().getRotation().getDegrees()));
        // the GOAL pose of robot is the target accounting for where
        // the robot should be relative to (in this case, 1.5m in front)
        var goalPose = targetPose.transformBy(tagToGoal).toPose2d();

        SmartDashboard.putString("Goal Pose", String.format("(%.2f, %.2f) %.2f degrees", 
          goalPose.getX(),
          goalPose.getY(), 
          goalPose.getRotation().getDegrees()));
        // set the x y and rot controllers to follow that goal
        xController.setGoal(goalPose.getX());
        yController.setGoal(goalPose.getY());
        // if (Math.abs(Rotation2d.fromRadians(rotController.getGoal().position).minus(goalPose.getRotation()).getRadians()) > 0.05) {
        //   rotController.setGoal(goalPose.getRotation().getRadians());
        // }
        rotController.setGoal(goalPose.getRotation().getRadians());
      }
    }
    if (lastTarget == null) {
      swerve.drive(0,0,0,true);
    } else {
      var xSpeed = xController.calculate(robotPose.getX());
      if (xController.atGoal()) {
        xSpeed = 0;
      }

      var ySpeed = yController.calculate(robotPose.getY());
      if (yController.atGoal()) {
        ySpeed = 0;
      }
      var rot = rotController.calculate(robotPose2d.getRotation().getRadians());
      if (rotController.atGoal()) {
        rot = 0;
      }
      swerve.drive(xSpeed, ySpeed, rot, true);
      
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(0,0,0,true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
