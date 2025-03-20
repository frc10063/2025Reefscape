// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignCommand extends Command {
  /** Creates a new AlignCommand. */
  private final DriveTrain m_swerve;
  private final VisionSubsystem m_vision;
  private static final TrapezoidProfile.Constraints xConstraints = new TrapezoidProfile.Constraints(3, 2);
  private static final TrapezoidProfile.Constraints yConstraints = new TrapezoidProfile.Constraints(3, 2);
  private static final TrapezoidProfile.Constraints rotConstraints = new TrapezoidProfile.Constraints(8, 8);

  private final ProfiledPIDController xController = new ProfiledPIDController(AutoConstants.translationkP, AutoConstants.translationkI, AutoConstants.translationkD, xConstraints);
  private final ProfiledPIDController yController = new ProfiledPIDController(AutoConstants.translationkP, AutoConstants.translationkI, AutoConstants.translationkD, yConstraints);
  private final ProfiledPIDController rotController = new ProfiledPIDController(AutoConstants.thetakP, AutoConstants.thetakI, AutoConstants.thetakD, rotConstraints);


  private Pose2d targetReefTagPose;
  private Pose2d targetPose;
  public AlignCommand(DriveTrain m_swerve, VisionSubsystem m_vision) {
    this.m_swerve = m_swerve;
    this.m_vision = m_vision;
    rotController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(m_swerve, m_vision);
  }

  @Override
  public void initialize() {
    xController.setTolerance(0.01, 0.01);
    yController.setTolerance(0.01, 0.01);
    rotController.setTolerance(0.01);

    targetReefTagPose = m_vision.findBestTagPose().toPose2d();

  }

  // this can be a later job
  private void calculateTargetPose() {
    // make a translation for offset and add the pose of target stuff
  }

  
  @Override
  public void execute() {
    Pose2d currentPose = m_swerve.getPose();
    Pose2d targetPose = targetReefTagPose;
    double xSpeed = xController.calculate(currentPose.getX(), targetPose.getX());
    double ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());
    double rot = rotController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    xSpeed = MathUtil.clamp(xSpeed, -AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxSpeedMetersPerSecond);
    ySpeed = MathUtil.clamp(ySpeed, -AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxSpeedMetersPerSecond);
    rot = MathUtil.clamp(rot, -AutoConstants.kMaxAngularSpeedRadiansPerSecond, AutoConstants.kMaxAngularSpeedRadiansPerSecond);
    // according to the teams code im getting inspiration from this should be fieldrelative
    m_swerve.drive(xSpeed, ySpeed, rot, true);
  }


 
  @Override
  public void end(boolean interrupted) {
    m_swerve.drive(0,0,0,true);
  }

  
  @Override
  public boolean isFinished() {
    return false;
  }
}
