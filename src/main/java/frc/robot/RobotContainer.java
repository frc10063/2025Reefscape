// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OperatorConstants;
// import frc.robot.commands.Autos;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ElevatorSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final XboxController m_controller = new XboxController(OperatorConstants.kXBoxControllerPort);
  private final CommandJoystick m_joystick = new CommandJoystick(OperatorConstants.kJoystickControllerPort);
  private final DriveTrain m_swerve = new DriveTrain();
  // private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  // private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  // private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kXBoxControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    m_swerve.setDefaultCommand(
        new RunCommand(
          () ->
              m_swerve.drive(
                  m_controller.getLeftY() * DriveConstants.kMaxSpeedMetersPerSecond,
                  m_controller.getLeftX() * DriveConstants.kMaxSpeedMetersPerSecond,
                  m_controller.getRightX() * ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond, 
                  false), 
                  m_swerve));
    m_elevatorSubsystem.setDefaultCommand(
        new RunCommand(
          () -> 
              m_elevatorSubsystem.moveElevator(
                m_joystick.getY()),
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
  private void configureBindings() {}

    
  // Commenting this out because I don't know if the slew rate stuff is necessary
  // private void driveWithJoystick(boolean fieldRelative) {
  //   // Get the x speed. We are inverting this because Xbox controllers return
  //   // negative values when we push forward.
  //   final var xSpeed =
  //       -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), 0.02))
  //           * DriveTrain.kMaxSpeed;

  //   // Get the y speed or sideways/strafe speed. We are inverting this because
  //   // we want a positive value when we pull to the left. Xbox controllers
  //   // return positive values when you pull to the right by default.
  //   final var ySpeed =
  //       -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftX(), 0.02))
  //           * DriveTrain.kMaxSpeed;

  //   // Get the rate of angular rotation. We are inverting this because we want a
  //   // positive value when we pull to the left (remember, CCW is positive in
  //   // mathematics). Xbox controllers return positive values when you pull to
  //   // the right by default.
  //   final var rot =
  //       -m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(), 0.02))
  //           * DriveTrain.kMaxAngularSpeed;

  //   m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative, getPeriod());
  // }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}
