// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
// import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public Autos(DriveTrain m_swerve, PoseEstimatorSubsystem m_poseEstimator) {
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }

    AutoBuilder.configure(
            m_poseEstimator::getPoseEstimate, // Robot pose supplier
            m_swerve::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            m_swerve::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> m_swerve.drive(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            m_swerve // Reference to this subsystem to set requirements
    );
  }
  public static Command coralPlacingAuto(ElevatorSubsystem m_elevatorSubsystem, EndEffectorSubsystem m_endEffectorSubsystem, String level) {
    return Commands.sequence(m_elevatorSubsystem.moveElevatorTo(level),
    m_endEffectorSubsystem.runEndEffector(),
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
    return new Move(m_swerve, -1.5, 0, new Rotation2d(0), true).withTimeout(2);
  }
  public static Command L1Sequence(DriveTrain m_swerve, EndEffectorSubsystem m_endEffectorSubsystem) {
    return Commands.run(() -> m_swerve.drive(0, -0.3, 0, false)).alongWith(m_endEffectorSubsystem.runL1EndEffector()).withTimeout(2);
  }
  public static Command VisionAlignAuto(DriveTrain m_swerve, ElevatorSubsystem m_elevatorSubsystem, EndEffectorSubsystem m_endEffectorSubsystem, PoseEstimatorSubsystem m_vision, boolean startLeft) {
    return Commands.sequence(
      new Move(m_swerve, -1, 0, new Rotation2d(startLeft ? -(2 * Math.PI)/3: (2 * Math.PI)/3), false), 
      new AlignCommand(m_swerve, m_vision, startLeft),
      m_elevatorSubsystem.moveElevatorTo("L2"), 
      m_endEffectorSubsystem.runEndEffector(),
      m_elevatorSubsystem.moveElevatorTo("ZERO")
    );
  }
  public static Command middleReefAuto(DriveTrain m_swerve, ElevatorSubsystem m_elevatorSubsystem, EndEffectorSubsystem m_endEffectorSubsystem, PoseEstimatorSubsystem m_vision) {
    return Commands.sequence(
      new Move(m_swerve, 0, 0, new Rotation2d(Math.PI/2), true), 
      new AlignCommand(m_swerve, m_vision, false), 
      m_elevatorSubsystem.moveElevatorTo("L1"),
      L1Sequence(m_swerve, m_endEffectorSubsystem)
    );
  }


  
}
