// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OperatorConstants;
// import frc.robot.commands.Autos;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  // private final IntakeSubsystem m_elevatorSubsystem = new IntakeSubsystem();
  private final CommandXboxController m_controller = new CommandXboxController(OperatorConstants.kXBoxControllerPort);
  private final CommandJoystick m_joystick = new CommandJoystick(OperatorConstants.kJoystickControllerPort);
  private final DriveTrain m_swerve = new DriveTrain();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  Trigger resetGyroTrigger = m_controller.a();
  Trigger setPointTrigger = m_joystick.button(2);
  Trigger halfSpeedTrigger = m_controller.rightTrigger();
  Trigger runIntakeTrigger = m_joystick.button(1);
  // private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  // private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  // private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    m_swerve.setDefaultCommand(
        new RunCommand(
          () ->
              m_swerve.drive(
                  MathUtil.applyDeadband(m_controller.getLeftY(), 0.05) * DriveConstants.kMaxSpeedMetersPerSecond, 
                  MathUtil.applyDeadband(m_controller.getLeftX(), 0.05) * DriveConstants.kMaxSpeedMetersPerSecond, 
                  MathUtil.applyDeadband(m_controller.getRightX(), 0.05) * ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond, 
                  true), 
                  m_swerve));
     m_elevatorSubsystem.setDefaultCommand(
        new RunCommand(
          () -> 
              m_elevatorSubsystem.moveElevator(
                MathUtil.applyDeadband(m_joystick.getY(), 0.1) * 0.6),
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
    setPointTrigger.whileTrue(new RunCommand(() -> m_elevatorSubsystem.setElevatorPosition(0)));
    halfSpeedTrigger.whileTrue(new StartEndCommand(m_swerve::setHalfSpeed, m_swerve::setDefaultSpeed, new Subsystem[0]));
    // runIntakeTrigger.whileTrue(new RunCommand(m_intakeSubsystem::runIntakeMaxSpeed));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    Trajectory toReefTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(0.3, 0),
                    new Translation2d(0.7, 0)),
            new Pose2d(1, 0, Rotation2d.fromRadians(Math.PI / 2)),
            config);
    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController = 
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            toReefTrajectory,
            m_swerve::getPose,
            DriveConstants.kDriveKinematics,
            xController,
            yController,
            thetaController,
            m_swerve::setModuleStates,
            m_swerve);

    // Reset odometry to the initial pose of the trajectory, run path following
    // command, then stop at the end.
    return Commands.sequence(
        new InstantCommand(() -> m_swerve.resetOdometry(toReefTrajectory.getInitialPose())),
        swerveControllerCommand,
        new InstantCommand(() -> m_swerve.drive(0, 0, 0, false)), 
        new RunCommand(m_intakeSubsystem::runIntakeMaxSpeed, m_intakeSubsystem));
  }
}
