// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public final class Autos {
  /** Example static factory for an autonomous command. */
  
  public static Command coralPlacingAuto(ElevatorSubsystem m_elevatorSubsystem, IntakeSubsystem m_intakeSubsystem, String level) {
    return Commands.sequence(m_elevatorSubsystem.moveElevatorTo(level),
    m_intakeSubsystem.runEndEffector(),
    m_elevatorSubsystem.moveElevatorTo("ZERO"));
  }


  public static Command DeAlgaeCommand(ElevatorSubsystem m_elevatorSubsystem, AlgaeSubsystem m_algaeSubsystem, String level) {
    return Commands.sequence(
      m_algaeSubsystem.extendAlgaeCommand(),
      m_elevatorSubsystem.moveElevatorTo(level), 
      Commands.waitSeconds(3),
      m_algaeSubsystem.retractAlgaeCommand(),
      m_elevatorSubsystem.moveElevatorTo("ZERO"));
  }
  public static Command taxi(DriveTrain m_swerve) {
    return new TaxiAuto(m_swerve).withTimeout(2);
  }


  
}
