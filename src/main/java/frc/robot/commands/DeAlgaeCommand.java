// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DeAlgaeCommand extends SequentialCommandGroup {
  /** Creates a new DeAlgaeCommand. */
  AlgaeSubsystem m_algaeSubsystem;
  ElevatorSubsystem m_elevatorSubsystem;
  String level;

  public DeAlgaeCommand(AlgaeSubsystem m_algaeSubsystem, ElevatorSubsystem m_elevatorSubsystem, String level) {
    this.m_algaeSubsystem = m_algaeSubsystem;
    this.m_elevatorSubsystem = m_elevatorSubsystem;
    this.level = level;
    
    addRequirements(m_algaeSubsystem, m_elevatorSubsystem);
    addCommands(new RunCommand(m_algaeSubsystem::extendAlgae).withTimeout(2)
    .andThen(m_elevatorSubsystem.moveElevatorTo(level),
        Commands.waitSeconds(3).andThen(new RunCommand(m_algaeSubsystem::retractAlgae).withTimeout(1.5))));
  }
}
