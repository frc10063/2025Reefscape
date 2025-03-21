// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class VisionLeftReefAuto extends SequentialCommandGroup {
  /** Creates a new VisionLeftReefAuto. */
  DriveTrain m_swerve;
  ElevatorSubsystem m_elevatorSubsystem;
  VisionSubsystem m_vision;
  IntakeSubsystem m_intakeSubsystem;
  RotationCommand rotLeftCommand;
  AlignCommand leftAlignCommand;
  CoralPlacingAuto coralPlacingAuto;
  
  public VisionLeftReefAuto() {
    rotLeftCommand = new RotationCommand(m_swerve, -Math.PI/3);
    leftAlignCommand = new AlignCommand(m_swerve, m_vision, false);
    coralPlacingAuto = new CoralPlacingAuto(m_elevatorSubsystem, m_intakeSubsystem, 2);
    // add to this for more coral later?
    addCommands(rotLeftCommand, leftAlignCommand, coralPlacingAuto);
  }
}
