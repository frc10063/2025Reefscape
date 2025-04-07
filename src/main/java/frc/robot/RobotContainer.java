// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
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
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
// import frc.robot.commands.ChaseTagCommand;
import frc.robot.commands.CoralPlacingAuto;
import frc.robot.commands.DeAlgaeCommand;
import frc.robot.commands.MoveForwardAuto;
import frc.robot.commands.RotationCommand;
import frc.robot.commands.TaxiAuto;
import frc.robot.commands.LeftReefAuto;
import frc.robot.commands.MiddleReefAuto;
import frc.robot.commands.RightReefAuto;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.Bongo;
import frc.robot.subsystems.DDRMat;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.subsystems.AlgaeSubsystem;
// import frc.robot.subsystems.VisionSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // OI
  private final CommandXboxController m_controller = new CommandXboxController(OperatorConstants.kXBoxControllerPort);
  private final CommandJoystick m_joystick = new CommandJoystick(OperatorConstants.kJoystickControllerPort);
  // private final Bongo m_bongoController = new Bongo(2);
  // private final DDRMat m_ddrController = new DDRMat(3);
  // slew rate limiters (optional)
  // private final SlewRateLimiter m_xLimiter = new SlewRateLimiter(3);
  // private final SlewRateLimiter m_yLimiter = new SlewRateLimiter(3);
  // private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);


  // Subsystems
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final DriveTrain m_swerve = new DriveTrain();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem();
  // private final VisionSubsystem m_vision = new VisionSubsystem(m_swerve);
  
  // Defining triggers

  // Joystick triggers
  Trigger runIntakeTrigger = m_joystick.button(1);
  // setpoints
  Trigger L1Trigger = m_joystick.button(2);
  Trigger L2Trigger = m_joystick.button(4);
  Trigger L3Trigger = m_joystick.button(5);
  Trigger L4Trigger = m_joystick.button(3);
  // Override for if elevator encoder readings are bad
  Trigger OverrideElevatorSafetyTrigger = m_joystick.button(9);

  Trigger forwardAlgaeTrigger = m_joystick.button(6);
  Trigger reverseAlgaeTrigger = m_joystick.button(7);
  
  Trigger deAlgaeL3Trigger = m_joystick.button(10);
  Trigger deAlgaeL2Trigger = m_joystick.button(11);

  // Controller triggers
  Trigger halfSpeedTrigger = m_controller.leftTrigger();
  Trigger fastSpeedTrigger = m_controller.rightTrigger();
  
  Trigger fieldRelativeHoldTrigger = m_controller.rightBumper();
  
  Trigger fieldRelativeToggleTrigger = m_controller.b();
  Trigger resetGyroTrigger = m_controller.y();

  // Bongo Triggers for fun
  // Trigger L1BongoTrigger = m_bongoController.getBottomLeft();
  // Trigger L2BongoTrigger = m_bongoController.getTopLeft();
  // Trigger L3BongoTrigger = m_bongoController.getBottomRight();
  // Trigger L4BongoTrigger = m_bongoController.getTopRight();

  // Trigger bongoPlaceL2Trigger = m_bongoController.getLeftFullBongo();
  // Trigger bongoPlaceL3Trigger = m_bongoController.getRightFullBongo();
  // Trigger bongoPlaceL4Trigger = m_bongoController.getLeftFullBongo().and(m_bongoController.getRightFullBongo());

  // Trigger clapIntakeTrigger = m_bongoController.getClap();

  // Trigger algaeToggleTrigger = m_bongoController.getMiddleButton();

  // DDRMat Triggers for Elevator / Intake
  // Trigger L1DDRTrigger = m_ddrController.getLeftArrow();
  // Trigger L2DDRTrigger = m_ddrController.getBlueUpArrow();
  // Trigger L3DDRTrigger = m_ddrController.getOrangeUpArrow();
  // Trigger L4DDR = m_ddrController.getRightArrow(); NO
  // Trigger DDRIntakeTrigger = m_ddrController.getRightArrow();
  // Trigger DDRIncreaseSpeedTrigger = m_ddrController.getPlusButton();
  // Trigger DDRDecreaseSpeedTrigger = m_ddrController.getMinusButton();

  // Vision align buttons, not tested
  // Trigger alignLeftCoralTrigger = m_controller.leftBumper();
  // Trigger alignRightCoralTrigger = m_controller.rightBumper();
  

  // Commands
  Command offLineAutoCommand = new MoveForwardAuto(m_swerve);
  RotationCommand rotMiddle90Command = new RotationCommand(m_swerve, -Math.PI/2);
  // for starting left side 
  RotationCommand rotLeftCommand = new RotationCommand(m_swerve, -Math.PI/3);
  // for right side
  RotationCommand rotRightCommand = new RotationCommand(m_swerve, -2 * Math.PI/3);
  // ChaseTagCommand chaseTagCommand = new ChaseTagCommand(m_vision, m_swerve);
  // Command leftAlignCommand  = new AlignCommand(m_swerve, m_vision, false);
  // Command rightAlignCommand = new AlignCommand(m_swerve, m_vision, true);
  Command L2DeAlgaeCommand = new DeAlgaeCommand(m_algaeSubsystem, m_elevatorSubsystem, 0);
  Command L3DeAlgaeCommand = new DeAlgaeCommand(m_algaeSubsystem, m_elevatorSubsystem, 1);
  Command coralPlacingCommand = new CoralPlacingAuto(m_elevatorSubsystem, m_intakeSubsystem, 2);
  Command taxiAutoCommand = new TaxiAuto(m_swerve);
  // Command visionLeftReefAuto = new VisionLeftReefAuto();
  // Command visionRightReefAuto = new VisionRightReefAuto();

  Command middleAutoCommand = new MiddleReefAuto(m_swerve, m_intakeSubsystem);
  Command leftAutoCommand = new LeftReefAuto(m_swerve, m_intakeSubsystem);
  Command rightAutoCommand = new RightReefAuto(m_swerve, m_intakeSubsystem);

  // chooser for autos
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  boolean fieldRelative = true;
  double DDRSpeedMultiplier = 1;

  // methods for enabling/disabling field relative from controller
  public void fieldRelativeToggle() {
    fieldRelative = !fieldRelative;
  }
  public void enableFieldRelative() {
    fieldRelative = true;
  }
  public void disableFieldRelative() {
    fieldRelative = false;
  }
  // ddr speed changing
  public void addDDRSpeed() {
    if (DDRSpeedMultiplier < 4) {
      DDRSpeedMultiplier = DDRSpeedMultiplier + 1;
    }
  }
  public void lowerDDRSpeed() {
    if (DDRSpeedMultiplier > 1) {
      DDRSpeedMultiplier = DDRSpeedMultiplier - 1;
    }
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    // options for sendable auto chooser
    m_chooser.setDefaultOption("Taxi Auto", taxiAutoCommand);
    m_chooser.addOption("Middle Reef Auto", middleAutoCommand);
    m_chooser.addOption("Left Reef Auto", leftAutoCommand);
    m_chooser.addOption("Right Reef Auto", rightAutoCommand);
    m_chooser.addOption("Real Middle Reef Auto", Commands.sequence(rotMiddle90Command, taxiAutoCommand.withTimeout(3), new CoralPlacingAuto(m_elevatorSubsystem, m_intakeSubsystem, 3)));
  
    SmartDashboard.putData(m_chooser);
    
    // Configure the trigger bindings

    configureBindings();
    

    // m_swerve.setDefaultCommand(
    //     new RunCommand(
    //       () ->
    //           m_swerve.drive(
    //               -MathUtil.applyDeadband(m_controller.getLeftY(), 0.05) * DriveConstants.kMaxSpeedMetersPerSecond, 
    //               -MathUtil.applyDeadband(m_controller.getLeftX(), 0.05) * DriveConstants.kMaxSpeedMetersPerSecond, 
    //               -MathUtil.applyDeadband(m_controller.getRightX(), 0.05) * DriveConstants.kMaxRotationSpeedRadiansPerSecond, 
    //               fieldRelative), 
    //               m_swerve));

    // Default commands run when nothing is scheduled
    // Tangent line applied to make smaller movements smaller, but maintain same max speed
    // As by default up is -y on a joystick and left is +x, joystick inputs must be inverted

    m_swerve.setDefaultCommand(
        new RunCommand(
          () ->
              m_swerve.drive(
                  Math.tan((Math.PI/4) * -MathUtil.applyDeadband(m_controller.getLeftY(), 0.05)) * DriveConstants.kMaxSpeedMetersPerSecond, 
                  Math.tan((Math.PI/4) * -MathUtil.applyDeadband(m_controller.getLeftX(), 0.05)) * DriveConstants.kMaxSpeedMetersPerSecond, 
                  Math.tan((Math.PI/4) * -MathUtil.applyDeadband(m_controller.getRightX(), 0.05)) * DriveConstants.kMaxRotationSpeedRadiansPerSecond, 
                  fieldRelative), 
                  m_swerve));
    // m_swerve.setDefaultCommand(
    //     new RunCommand(
    //       () ->
    //           m_swerve.drive(
    //             m_ddrController.getMatYValue() * DDRSpeedMultiplier, 
    //             m_ddrController.getMatXValue() * DDRSpeedMultiplier, 
    //             m_ddrController.getMatRotValue() * DDRSpeedMultiplier,
    //             fieldRelative),
    //           m_swerve));
    
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

    // Controller bindings
    // zero gyro
    resetGyroTrigger.onTrue(new InstantCommand(m_swerve::zeroHeading));
    
    fieldRelativeToggleTrigger.onTrue(new InstantCommand(this::fieldRelativeToggle));
    fieldRelativeHoldTrigger.whileTrue(new StartEndCommand(this::disableFieldRelative, this::enableFieldRelative, new Subsystem[0]));

    halfSpeedTrigger.whileTrue(new StartEndCommand(m_swerve::slowSpeed, m_swerve::defaultSpeed, new Subsystem[0]));
    fastSpeedTrigger.whileTrue(new StartEndCommand(m_swerve::fastSpeed, m_swerve::defaultSpeed, new Subsystem[0]));

    // alignRightCoralTrigger.whileTrue(rightAlignCommand);
    // alignLeftCoralTrigger.whileTrue(leftAlignCommand);

    // Joystick bindings
    L1Trigger.whileTrue(new RunCommand(() -> m_elevatorSubsystem.setElevatorPosition(ElevatorConstants.kElevatorSetpoints[0]), m_elevatorSubsystem));
    L2Trigger.whileTrue(new RunCommand(() -> m_elevatorSubsystem.setElevatorPosition(ElevatorConstants.kElevatorSetpoints[1]), m_elevatorSubsystem));
    L3Trigger.whileTrue(new RunCommand(() -> m_elevatorSubsystem.setElevatorPosition(ElevatorConstants.kElevatorSetpoints[2]), m_elevatorSubsystem));
    L4Trigger.whileTrue(new RunCommand(() -> m_elevatorSubsystem.setElevatorPosition(ElevatorConstants.kElevatorSetpoints[3]), m_elevatorSubsystem));

    runIntakeTrigger.whileTrue(new StartEndCommand(m_intakeSubsystem::runIntakeMaxSpeed, m_intakeSubsystem::stopIntake, m_intakeSubsystem));

    forwardAlgaeTrigger.whileTrue(new StartEndCommand(m_algaeSubsystem::runAlgaeMaxSpeed, m_algaeSubsystem::stopAlgae, m_algaeSubsystem));
    reverseAlgaeTrigger.whileTrue(new StartEndCommand(m_algaeSubsystem::reverseAlgaeMaxSpeed, m_algaeSubsystem::stopAlgae, m_algaeSubsystem));

    deAlgaeL2Trigger.onTrue(new RunCommand(m_algaeSubsystem::extendAlgae, m_algaeSubsystem).withTimeout(1.2).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    deAlgaeL3Trigger.onTrue(new RunCommand(m_algaeSubsystem::retractAlgae, m_algaeSubsystem).withTimeout(1.2).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    
    OverrideElevatorSafetyTrigger.onTrue(new InstantCommand(m_elevatorSubsystem::overrideElevatorSafety));

    // Bongo
    // L1BongoTri?gger.whileTrue(new RunCommand(() -> m_elevatorSubsystem.setElevatorPosition(0), m_elevatorSubsystem));
    // L2BongoTrigger.whileTrue(new RunCommand(() -> m_elevatorSubsystem.setElevatorPosition(ElevatorConstants.kElevatorSetpoints[1]), m_elevatorSubsystem));
    // L3BongoTrigger.whileTrue(new RunCommand(() -> m_elevatorSubsystem.setElevatorPosition(ElevatorConstants.kElevatorSetpoints[2]), m_elevatorSubsystem));
    // L4BongoTrigger.whileTrue(new RunCommand(() -> m_elevatorSubsystem.setElevatorPosition(ElevatorConstants.kElevatorSetpoints[3]), m_elevatorSubsystem));
    
    // bongoPlaceL2Trigger.onTrue(new CoralPlacingAuto(m_elevatorSubsystem, m_intakeSubsystem, 2));
    // bongoPlaceL3Trigger.onTrue(new CoralPlacingAuto(m_elevatorSubsystem, m_intakeSubsystem, 3));
    //bongoPlaceL4Trigger.onTrue(new CoralPlacingAuto(m_elevatorSubsystem, m_intakeSubsystem, 4).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    // clapIntakeTrigger.onTrue(new StartEndCommand(m_intakeSubsystem::runIntakeMaxSpeed, m_intakeSubsystem::stopIntake, m_intakeSubsystem).withTimeout(0.5).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    // algaeToggleTrigger.onTrue(new StartEndCommand(m_algaeSubsystem::toggleAlgae, m_algaeSubsystem::stopAlgae, m_algaeSubsystem).withTimeout(0.5).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    // L4BongoTrigger.whileTrue(new RunCommand(() -> m_elevatorSubsystem.setElevatorPosition(ElevatorConstants.kElevatorSetpoints[3]), m_elevatorSubsystem));

    // DDRMat (was originally gonna make it drive but scared)
    // DDRL1Trigger.whileTrue(new RunCommand(() -> m_elevatorSubsystem.setElevatorPosition(ElevatorConstants.kElevatorSetpoints[0]), m_elevatorSubsystem));
    // DDRL2Trigger.whileTrue(new RunCommand(() -> m_elevatorSubsystem.setElevatorPosition(ElevatorConstants.kElevatorSetpoints[1]), m_elevatorSubsystem));
    // DDRL3Trigger.whileTrue(new RunCommand(() -> m_elevatorSubsystem.setElevatorPosition(ElevatorConstants.kElevatorSetpoints[2]), m_elevatorSubsystem));

    // DDRIntakeTrigger.onTrue(new RunCommand(m_intakeSubsystem::runIntakeMaxSpeed, m_intakeSubsystem).withTimeout(0.5));
    // DDRL4Trigger.whileTrue(new RunCommand(() -> m_elevatorSubsystem.setElevatorPosition(ElevatorConstants.kElevatorSetpoints[3]), m_elevatorSubsystem));
    // DDRIncreaseSpeedTrigger.onTrue(new InstantCommand(this::addDDRSpeed));
    // DDRDecreaseSpeedTrigger.onTrue(new InstantCommand(this::lowerDDRSpeed));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
    // test this (commented out for now -b)
    // return coralPlacingCommand;

    // just comment and uncomment
    // return visionLeftReefAuto;
    // return visionRightReefAuto;
    // return VisionMiddleReefAuto;

    // if all goes wrong use this
    // return taxiAutoCommand.withTimeout(2);
    // return new RunCommand(() -> m_swerve.drive(-5, 0, 0, true)).withTimeout(2);
  }
}
