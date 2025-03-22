// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TaxiAuto extends Command {
  /** Creates a new TaxiAuto. */
  private final DriveTrain m_swerve;
  private static final TrapezoidProfile.Constraints xConstraints = new TrapezoidProfile.Constraints(2, 3);
  private final ProfiledPIDController xController = new ProfiledPIDController(1, 0, 0, xConstraints);
  public TaxiAuto(DriveTrain m_swerve) {
    this.m_swerve = m_swerve;
    addRequirements(m_swerve);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xController.reset(m_swerve.getPose().getX());
    xController.setTolerance(0.1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = xController.calculate(m_swerve.getPose().getX(), -3);
    m_swerve.drive(xSpeed, 0, 0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
