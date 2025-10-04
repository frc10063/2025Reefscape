// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveTrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Move extends Command {
  private final DriveTrain m_swerve;
  private static final TrapezoidProfile.Constraints xConstraints = new TrapezoidProfile.Constraints(2, 3);
  private static final TrapezoidProfile.Constraints yConstraints = new TrapezoidProfile.Constraints(2, 3);
  private static final TrapezoidProfile.Constraints rotConstraints = new TrapezoidProfile.Constraints(2 * Math.PI, 2 * Math.PI);
  private final ProfiledPIDController rotController = new ProfiledPIDController(3, 0, 0, rotConstraints);
  private final ProfiledPIDController xController = new ProfiledPIDController(AutoConstants.translationkP, AutoConstants.translationkI, AutoConstants.translationkD, xConstraints);
  private final ProfiledPIDController yController = new ProfiledPIDController(AutoConstants.translationkP, AutoConstants.translationkI, AutoConstants.translationkD, yConstraints);
  private double xDistance;
  private double yDistance;
  private Rotation2d targetAngle;
  private boolean fieldRelative;
  public Move(DriveTrain m_swerve, double xDistance, double yDistance, Rotation2d targetAngle, boolean fieldRelative) {
    this.m_swerve = m_swerve;
    this.xDistance = xDistance;
    this.yDistance = yDistance;
    this.targetAngle = targetAngle;
    this.fieldRelative = fieldRelative;
    addRequirements(m_swerve);
  }

  
  @Override
  public void initialize() {
    xController.reset(m_swerve.getPose().getX());
    xController.setTolerance(0.1);
    yController.reset(m_swerve.getPose().getY());
    yController.setTolerance(0.1);
    rotController.reset(m_swerve.getPose().getRotation().getRadians());
    rotController.setTolerance(Units.degreesToRadians(3));
    rotController.enableContinuousInput(-Math.PI, Math.PI);

    xController.setGoal(xDistance);
    yController.setGoal(yDistance);
    rotController.setGoal(targetAngle.getRadians());
  }

  
  @Override
  public void execute() {
    double xSpeed = xController.calculate(m_swerve.getPose().getX());
    double ySpeed = yController.calculate(m_swerve.getPose().getY());
    double rotSpeed = rotController.calculate(m_swerve.getPose().getRotation().getRadians());
    m_swerve.drive(xSpeed, ySpeed, rotSpeed, fieldRelative);
  }

  
  @Override
  public void end(boolean interrupted) {
    m_swerve.drive(0, 0, 0, fieldRelative);
  }

  
  @Override
  public boolean isFinished() {
    return xController.atGoal() && yController.atGoal() && rotController.atGoal();
  }
}
