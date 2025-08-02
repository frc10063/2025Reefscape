// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PoseEstimatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToTagCommand extends Command {
  /** Creates a new AlignToTagCommand. */
  private static final TrapezoidProfile.Constraints xConstraints = new TrapezoidProfile.Constraints(0.5, 0.5);
  private static final TrapezoidProfile.Constraints yConstraints = new TrapezoidProfile.Constraints(0.3, 0.1);
  private static final TrapezoidProfile.Constraints rotConstraints = new TrapezoidProfile.Constraints(Math.PI, Math.PI);

  private static final int tagToChase = 15;
//   private final PhotonCamera photonCamera;
  private final DriveTrain swerve;
  private final PoseEstimatorSubsystem vision;
  private PhotonTrackedTarget lastTarget;
  PhotonTrackedTarget target;
  private double targetPitch;
  private double targetAngle;

  private final ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, xConstraints);
  private final ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, yConstraints);
  private final ProfiledPIDController rotController = new ProfiledPIDController(3, 0, 0, rotConstraints);
  public AlignToTagCommand(PoseEstimatorSubsystem vision, DriveTrain swerve) {
    this.swerve = swerve;
    this.vision = vision;

    xController.setTolerance(0.1);
    yController.setTolerance(0.1);
    rotController.setTolerance(Units.degreesToRadians(3));
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
    PhotonPipelineResult photonRes = vision.getNewestResult();
    if (photonRes.hasTargets()) {
      var targetOpt = photonRes.getTargets().stream()
          .filter(t -> t.getFiducialId() == tagToChase)
          .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= 0.2) // && t.getPose
          .findFirst();
      if (targetOpt.isPresent()) {
        target = targetOpt.get();
        lastTarget = target;

        targetAngle = target.getYaw();
        targetPitch = target.getPitch();

        rotController.setGoal(0);
        xController.setGoal(0.5);
      }
    }
    if (lastTarget == null) {
      swerve.drive(0,0,0,true);
    } else {
      double rot = rotController.calculate(Units.degreesToRadians(targetAngle));
      if (rotController.atGoal()) {
        rot = 0;
      }
      // maybe make this negative?
      double xSpeed = xController.calculate(target.getBestCameraToTarget().getX());
      if (xController.atGoal()) {
        xSpeed = 0;
      }
      swerve.drive(xSpeed, 0, rot, true);
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
