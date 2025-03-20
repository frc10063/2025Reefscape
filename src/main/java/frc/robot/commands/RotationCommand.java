// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RotationCommand extends Command {
  /** Creates a new RotationCommand. */
  private final DriveTrain m_swerve;
  private static final TrapezoidProfile.Constraints rotConstraints = new TrapezoidProfile.Constraints(8, 8);
  private final ProfiledPIDController rotController = new ProfiledPIDController(1, 0, 0, rotConstraints);
  private final double targetAngle;

  public RotationCommand(DriveTrain m_swerve, double angle) {
    this.m_swerve = m_swerve;
    this.targetAngle = m_swerve.getGyroRotation().getRadians() + angle;
    addRequirements(m_swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotController.setTolerance(Units.degreesToRadians(3));
    rotController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotSpeed = rotController.calculate(m_swerve.getGyroRotation().getRadians(), targetAngle);
    m_swerve.drive(0, 0, rotSpeed, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.drive(0,0,0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return rotController.atGoal();
  }
}
