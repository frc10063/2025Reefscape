// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ChaseTagCommand;
import frc.robot.commands.CoralPlacingAuto;
import frc.robot.commands.LeftReefAuto;
import frc.robot.commands.MiddleReefAuto;
import frc.robot.commands.MoveForwardAuto;
import frc.robot.commands.RightReefAuto;
import frc.robot.commands.RotationCommand;
import frc.robot.commands.VisionLeftReefAuto;
import frc.robot.commands.VisionMiddleReefAuto;
import frc.robot.commands.VisionRightReefAuto;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.Autos;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.VisionSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final CommandXboxController m_controller = new CommandXboxController(OperatorConstants.kXBoxControllerPort);
  private final CommandJoystick m_joystick = new CommandJoystick(OperatorConstants.kJoystickControllerPort);
  private final DriveTrain m_swerve = new DriveTrain();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  // private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem();
  private final VisionSubsystem m_vision = new VisionSubsystem(m_swerve);
  private final SlewRateLimiter m_xLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
  Trigger resetGyroTrigger = m_controller.y();
  Trigger L1Trigger = m_joystick.button(2);
  Trigger L2Trigger = m_joystick.button(4);
  Trigger L3Trigger = m_joystick.button(5);
  Trigger L4Trigger = m_joystick.button(3);
  Trigger OverrideElevatorSafetyTrigger = m_joystick.button(7);
  Trigger halfSpeedTrigger = m_controller.rightTrigger();
  Trigger runIntakeTrigger = m_joystick.button(1);
  Trigger runLeftAlgaeTrigger = m_joystick.button(8);
  Trigger runRightAlgaeTrigger = m_joystick.button(9);
  Trigger alignLeftCoralTrigger = m_controller.leftBumper();
  Trigger alignRightCoralTrigger = m_controller.rightBumper();
  Command offLineAutoCommand = new MoveForwardAuto(m_swerve);
  RotationCommand rotMiddle90Command = new RotationCommand(m_swerve, -Math.PI/2);
  // for starting left side 
  RotationCommand rotLeftCommand = new RotationCommand(m_swerve, -Math.PI/3);
  // for right side
  RotationCommand rotRightCommand = new RotationCommand(m_swerve, -2 * Math.PI/3);
  ChaseTagCommand chaseTagCommand = new ChaseTagCommand(m_vision, m_swerve);
  Command leftAlignCommand  = new AlignCommand(m_swerve, m_vision, false);
  Command rightAlignCommand = new AlignCommand(m_swerve, m_vision, true);
  Command coralPlacingCommand = new CoralPlacingAuto(m_elevatorSubsystem, m_intakeSubsystem, 2);
  
  Command visionLeftReefAuto = new VisionLeftReefAuto();
  Command visionRightReefAuto = new VisionRightReefAuto();

  // Command middleAutoCommand = new MiddleReefAuto(m_swerve, m_intakeSubsystem);
  // Command leftAutoCommand = new LeftReefAuto(m_swerve, m_intakeSubsystem);
  // Command rightAutoCommand = new RightReefAuto(m_swerve, m_intakeSubsystem);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings


    configureBindings();
    
    m_swerve.setDefaultCommand(
        new RunCommand(
          () ->
              m_swerve.drive(
                  -MathUtil.applyDeadband(m_controller.getLeftY(), 0.1) * DriveConstants.kMaxSpeedMetersPerSecond, 
                  -MathUtil.applyDeadband(m_controller.getLeftX(), 0.1) * DriveConstants.kMaxSpeedMetersPerSecond, 
                  -MathUtil.applyDeadband(m_controller.getRightX(), 0.1) * DriveConstants.kMaxRotationSpeedRadiansPerSecond, 
                  true), 
                  m_swerve));
     m_elevatorSubsystem.setDefaultCommand(
        new RunCommand(
          () -> 
              m_elevatorSubsystem.moveElevator(
                MathUtil.applyDeadband(-m_joystick.getY(), 0.2) * 0.5),
                m_elevatorSubsystem));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    resetGyroTrigger.onTrue(new InstantCommand(m_swerve::zeroHeading));

    L1Trigger.whileTrue(new RunCommand(() -> m_elevatorSubsystem.setElevatorPosition(ElevatorConstants.kElevatorSetpoints[0]), m_elevatorSubsystem));
    L2Trigger.whileTrue(new RunCommand(() -> m_elevatorSubsystem.setElevatorPosition(ElevatorConstants.kElevatorSetpoints[1]), m_elevatorSubsystem));
    L3Trigger.whileTrue(new RunCommand(() -> m_elevatorSubsystem.setElevatorPosition(ElevatorConstants.kElevatorSetpoints[2]), m_elevatorSubsystem));
    L4Trigger.whileTrue(new RunCommand(() -> m_elevatorSubsystem.setElevatorPosition(ElevatorConstants.kElevatorSetpoints[3]), m_elevatorSubsystem));

    OverrideElevatorSafetyTrigger.onTrue(new InstantCommand(m_elevatorSubsystem::overrideElevatorSafety));

    halfSpeedTrigger.whileTrue(new StartEndCommand(m_swerve::slowSpeed, m_swerve::defaultSpeed, new Subsystem[0]));
    runIntakeTrigger.whileTrue(new StartEndCommand(m_intakeSubsystem::runIntakeMaxSpeed, m_intakeSubsystem::stopIntake, m_intakeSubsystem));

    alignRightCoralTrigger.whileTrue(rightAlignCommand);
    alignLeftCoralTrigger.whileTrue(leftAlignCommand);

    // runLeftAlgaeTrigger.whileTrue(new StartEndCommand(m_algaeSubsystem::runAlgaeMaxSpeed, m_algaeSubsystem::stopAlgae, m_algaeSubsystem));
    // runRightAlgaeTrigger.whileTrue(new StartEndCommand(m_algaeSubsystem::reverseAlgaeMaxSpeed, m_algaeSubsystem::stopAlgae, m_algaeSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // test this
    return coralPlacingCommand;

    // too much work to do chooser on dahsboard now
    // just comment and uncomment
    // return visionLeftReefAuto;
    // return visionRightReefAuto;
    // return VisionMiddleReefAuto;
    
    // if all goes wrong use this
    // return offLineAutoCommand;
  }
}
