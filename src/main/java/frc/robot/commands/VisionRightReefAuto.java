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
public class VisionRightReefAuto extends SequentialCommandGroup {
  /** Creates a new VisionRightReefAuto. */
  DriveTrain m_swerve;
  ElevatorSubsystem m_elevatorSubsystem;
  VisionSubsystem m_vision;
  IntakeSubsystem m_intakeSubsystem;
  RotationCommand rotRightCommand;
  AlignCommand leftAlignCommand;
  CoralPlacingAuto coralPlacingAuto;
  public VisionRightReefAuto() {
    rotRightCommand = new RotationCommand(m_swerve, -2 * Math.PI/3);
    leftAlignCommand = new AlignCommand(m_swerve, m_vision, false);
    coralPlacingAuto = new CoralPlacingAuto(m_elevatorSubsystem, m_intakeSubsystem, 2);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(rotRightCommand, leftAlignCommand, coralPlacingAuto);
  }
}
