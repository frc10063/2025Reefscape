// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CoralPlacingAuto extends SequentialCommandGroup {
  /** Creates a new CoralPlacingAuto. */
  ElevatorSubsystem m_elevatorSubsystem;
  IntakeSubsystem m_intakeSubsystem;
  String level;
  
  public CoralPlacingAuto(ElevatorSubsystem m_elevatorSubsystem, IntakeSubsystem m_intakeSubsystem, String level) {
    this.m_elevatorSubsystem = m_elevatorSubsystem;
    this.m_intakeSubsystem = m_intakeSubsystem;
    this.level = level;

    addCommands(m_elevatorSubsystem.moveElevatorTo(level), new StartEndCommand(m_intakeSubsystem::runIntakeMaxSpeed, m_intakeSubsystem::stopIntake, m_intakeSubsystem).withTimeout(1),
      m_elevatorSubsystem.moveElevatorTo("ZERO"));
  }
}
