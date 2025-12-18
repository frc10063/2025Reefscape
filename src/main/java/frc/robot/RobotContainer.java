// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import static frc.robot.Constants.DriveConstants.LINEAR_SPEED;
import static frc.robot.Constants.DriveConstants.MAX_ANGULAR_VELOCITY;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.ChaseTagCommand;
import frc.robot.commands.LeftReefAuto;
import frc.robot.commands.MiddleReefAuto;
import frc.robot.commands.Move;
import frc.robot.commands.RightReefAuto;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
// import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.controllers.Bongo;
import frc.robot.subsystems.controllers.DDRMat;
import frc.robot.subsystems.controllers.WiiBalanceBoard;
import frc.robot.subsystems.controllers.WiiBalanceBoard;



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
  private final Bongo m_bongoController = new Bongo(OperatorConstants.kBongoControllerPort);
  private final DDRMat m_ddrController = new DDRMat(OperatorConstants.kDDRControllerPort);
  private final XboxController m_EEGHeadset = new XboxController(OperatorConstants.kHeadsetPort);
  private final WiiBalanceBoard m_balanceBoard = new WiiBalanceBoard(OperatorConstants.kBalanceBoardPort);

  // Subsystems
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final DriveTrain m_swerve = new DriveTrain();
  private final EndEffectorSubsystem m_endEffectorSubsystem = new EndEffectorSubsystem(m_elevatorSubsystem);
  private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem();
  private final PoseEstimatorSubsystem m_vision = new PoseEstimatorSubsystem(m_swerve);
  



  // Joystick triggers
  Trigger runIntakeTrigger = m_joystick.button(1);

  // Setpoints
  Trigger L1Trigger = m_joystick.button(2);
  Trigger L2Trigger = m_joystick.button(4);
  Trigger L3Trigger = m_joystick.button(5);
  Trigger L4Trigger = m_joystick.button(3);

  // Override for if elevator encoder readings are bad
  Trigger OverrideElevatorSafetyTrigger = m_joystick.button(9);
  Trigger coralManipulatingTrigger = m_joystick.button(8);

  Trigger forwardAlgaeTrigger = m_joystick.button(6);
  Trigger reverseAlgaeTrigger = m_joystick.button(7);
  
  Trigger deAlgaeL3Trigger = m_joystick.button(10);
  Trigger deAlgaeL2Trigger = m_joystick.button(11);

  // Controller triggers
  Trigger halfSpeedTrigger = m_controller.leftTrigger();
  Trigger fastSpeedTrigger = m_controller.rightTrigger();
  Trigger speedChangeTrigger = m_controller.leftTrigger(0.01).or(m_controller.rightTrigger(0.01));
  
  Trigger fieldRelativeHoldTrigger = m_controller.a();
  
  Trigger fieldRelativeToggleTrigger = m_controller.b();
  Trigger resetGyroTrigger = m_controller.y();
  Trigger controllerSwapTrigger = m_controller.x();

  // Bongo Triggers for fun
  Trigger L1BongoTrigger = m_bongoController.getBottomLeft();
  Trigger L2BongoTrigger = m_bongoController.getTopLeft();
  Trigger L3BongoTrigger = m_bongoController.getBottomRight();
  Trigger L4BongoTrigger = m_bongoController.getTopRight();

  Trigger bongoPlaceL2Trigger = m_bongoController.getLeftFullBongo();
  Trigger bongoPlaceL3Trigger = m_bongoController.getRightFullBongo();
  Trigger bongoZeroTrigger = m_bongoController.getBottomLeft().and(m_bongoController.getBottomRight());
 // Trigger bongoPlaceL4Trigger = m_bongoController.getLeftFullBongo().and(m_bongoController.getRightFullBongo());

  Trigger clapIntakeTrigger = m_bongoController.getClap();

  Trigger algaeToggleTrigger = m_bongoController.getMiddleButton();

  // DDRMat Triggers for Elevator / Intake
  // Trigger L1DDRTrigger = m_ddrController.getLeftArrow();
  // Trigger L2DDRTrigger = m_ddrController.getBlueUpArrow();
  // Trigger L3DDRTrigger = m_ddrController.getOrangeUpArrow();
  // Trigger L4DDR = m_ddrController.getRightArrow(); NO
  // Trigger DDRIntakeTrigger = m_ddrController.getRightArrow();
  Trigger DDRIncreaseSpeedTrigger = m_ddrController.getPlusButton();
  Trigger DDRDecreaseSpeedTrigger = m_ddrController.getMinusButton();

  // Vision align buttons, not tested
  Trigger alignLeftCoralTrigger = m_controller.leftBumper();
  Trigger alignRightCoralTrigger = m_controller.rightBumper();
  
  Trigger balanceBoardCalibrate = m_balanceBoard.calibrationButton();

  // Commands
  Move rotMiddle90Command = new Move(m_swerve, 0, 0, new Rotation2d(-Math.PI/2), true);
  // for starting left side 
  Move rotLeftCommand = new Move(m_swerve, 0, 0, new Rotation2d(-Math.PI/3), true);
  // for right side
  Move rotRightCommand = new Move(m_swerve, 0, 0, new Rotation2d(Math.PI/3), true);
  ChaseTagCommand chaseTagCommand = new ChaseTagCommand(m_vision, m_swerve);
  Command leftAlignCommand  = new AlignCommand(m_swerve, m_vision, false);
  Command rightAlignCommand = new AlignCommand(m_swerve, m_vision, true);
  
  // Command taxiAutoCommand = new Move(m_swerve, -2, 0, true);

  Command middleAutoCommand = new MiddleReefAuto(m_swerve, m_endEffectorSubsystem);
  Command leftAutoCommand = new LeftReefAuto(m_swerve, m_endEffectorSubsystem);
  Command rightAutoCommand = new RightReefAuto(m_swerve, m_endEffectorSubsystem);

  
  // chooser for autos
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  SendableChooser<Command> m_driveChooser = new SendableChooser<>();
  private Command activeDriveCommand = null;
  // SendableChooser<Runnable> m_operatorChooser = new SendableChooser<>();

  boolean fieldRelative = true;
  double DDRSpeedMultiplier = 1;
  boolean swerveController = true;
  Command ddrCommand = new RunCommand(
    () ->
      m_swerve.drive(
        m_ddrController.getMatYValue() * DDRSpeedMultiplier, 
        m_ddrController.getMatXValue() * DDRSpeedMultiplier, 
        m_ddrController.getMatRotValue() * DDRSpeedMultiplier,
        fieldRelative),
      m_swerve);
  Command controllerCommand = new RunCommand(
    () ->
      m_swerve.drive(
        Math.tan((Math.PI/4) * -MathUtil.applyDeadband(m_controller.getLeftY(), 0.1)) * DriveConstants.LINEAR_SPEED, 
        Math.tan((Math.PI/4) * -MathUtil.applyDeadband(m_controller.getLeftX(), 0.1)) * DriveConstants.LINEAR_SPEED, 
        Math.tan((Math.PI/4) * -MathUtil.applyDeadband(m_controller.getRightX(), 0.1)) * DriveConstants.MAX_ANGULAR_VELOCITY,
        fieldRelative),
      m_swerve);
  Command wiiBalanceCommand = new RunCommand(
    () -> 
        m_swerve.drive(
            m_balanceBoard.applyResponseCurve(m_balanceBoard.getYAxis()) * DriveConstants.LINEAR_SPEED,
            -m_balanceBoard.applyResponseCurve(m_balanceBoard.getXAxis()) * DriveConstants.LINEAR_SPEED,
            m_balanceBoard.applyResponseCurve(m_balanceBoard.getRotAxis()) * DriveConstants.MAX_ANGULAR_VELOCITY,
            fieldRelative
        ),
    m_swerve);
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
    if (DDRSpeedMultiplier < 2) {
      DDRSpeedMultiplier = DDRSpeedMultiplier + 0.2;
    }
  }
  public void lowerDDRSpeed() {
    if (DDRSpeedMultiplier > 0.4) {
      DDRSpeedMultiplier = DDRSpeedMultiplier - 0.2;
    }
  }
  



  public RobotContainer() {
    
    // options for sendable auto chooser
    m_chooser.setDefaultOption("Taxi Auto", Autos.taxi(m_swerve));
    m_chooser.addOption("Middle Reef Auto", Autos.middleReefAuto(m_swerve, m_elevatorSubsystem, m_endEffectorSubsystem, m_vision));
    m_chooser.addOption("Left Reef Auto", Autos.VisionAlignAuto(m_swerve, m_elevatorSubsystem, m_endEffectorSubsystem, m_vision, true));
    m_chooser.addOption("Right Reef Auto", Autos.VisionAlignAuto(m_swerve, m_elevatorSubsystem, m_endEffectorSubsystem, m_vision, false));
    //m_chooser.addOption("Real Middle Reef Auto", Commands.sequence(rotMiddle90Command, taxiAutoCommand.withTimeout(3), Autos.coralPlacingAuto(m_elevatorSubsystem, m_endEffectorSubsystem, "L2")));

    m_driveChooser.setDefaultOption("Xbox Controller", controllerCommand);
    m_driveChooser.addOption("DDR Mat", ddrCommand);
    m_driveChooser.addOption("Balance Board", wiiBalanceCommand);


    SmartDashboard.putData(m_chooser);
    // SmartDashboard.putData("Drive Controller", m_driveChooser);
    DriverStation.silenceJoystickConnectionWarning(true);
    
    configureBindings();

    // Default commands run when nothing is scheduled
    // Tangent line applied to make smaller movements smaller, but maintain same max speed
    // As by default up is -y on a joystick and left is +x, joystick inputs must be inverted

    m_swerve.setDefaultCommand(controllerCommand);
    // m_swerve.setDefaultCommand(
    //     new RunCommand(
    //       () ->
    //           m_swerve.drive(
    //               Math.tan((Math.PI/4) * -MathUtil.applyDeadband(m_controller.getLeftY(), 0.1)) * DriveConstants.LINEAR_SPEED, 
    //               Math.tan((Math.PI/4) * -MathUtil.applyDeadband(m_controller.getLeftX(), 0.1)) * DriveConstants.LINEAR_SPEED, 
    //               Math.tan((Math.PI/4) * -MathUtil.applyDeadband(m_controller.getRightX(), 0.1)) * DriveConstants.MAX_ANGULAR_VELOCITY, 
    //               fieldRelative), 
    //               m_swerve));

    // m_swerve.setDefaultCommand(
    //       new RunCommand(
    //         () -> 
    //             m_swerve.drive(
    //                 m_balanceBoard.getYAxis(),
    //                 m_balanceBoard.getXAxis(),
    //                 m_balanceBoard.getRotAxis(),
    //                 fieldRelative
    //             ),
    //         m_swerve));
    // m_swerve.setDefaultCommand(
    //     new RunCommand(
    //       () ->
    //           m_swerve.drive(
    //             m_ddrController.getMatYValue() * DDRSpeedMultiplier, 
    //             m_ddrController.getMatXValue() * DDRSpeedMultiplier, 
    //             m_ddrController.getMatRotValue() * DDRSpeedMultiplier,
    //             fieldRelative),
    //           m_swerve));

    // m_swerve.setDefaultCommand(wiiBalanceCommand);
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
    
    // fieldRelativeToggleTrigger.onTrue(new InstantCommand(this::fieldRelativeToggle));
    fieldRelativeToggleTrigger.onTrue(controllerCommand.withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    fieldRelativeHoldTrigger.whileTrue(new StartEndCommand(this::disableFieldRelative, this::enableFieldRelative, new Subsystem[0]));

    halfSpeedTrigger.whileTrue(new StartEndCommand(m_swerve::slowSpeed, m_swerve::defaultSpeed, new Subsystem[0]));
    fastSpeedTrigger.whileTrue(new StartEndCommand(m_swerve::fastSpeed, m_swerve::defaultSpeed, new Subsystem[0]));

    controllerSwapTrigger.onTrue(ddrCommand.withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    
    alignRightCoralTrigger.whileTrue(rightAlignCommand);
    alignLeftCoralTrigger.whileTrue(leftAlignCommand);



    // Joystick bindings
    L1Trigger.onTrue(m_elevatorSubsystem.moveElevatorTo("L1"));
    L2Trigger.onTrue(m_elevatorSubsystem.moveElevatorTo("L2"));
    L3Trigger.onTrue(m_elevatorSubsystem.moveElevatorTo("L3"));
    // L4Trigger.onTrue(m_elevatorSubsystem.moveElevatorTo("L4"));


    runIntakeTrigger.onTrue(m_endEffectorSubsystem.runEndEffector());


    forwardAlgaeTrigger.whileTrue(new StartEndCommand(m_algaeSubsystem::runAlgaeMaxSpeed, m_algaeSubsystem::stopAlgae, m_algaeSubsystem));
    reverseAlgaeTrigger.whileTrue(new StartEndCommand(m_algaeSubsystem::reverseAlgaeMaxSpeed, m_algaeSubsystem::stopAlgae, m_algaeSubsystem));


    deAlgaeL2Trigger.onTrue(new RunCommand(m_algaeSubsystem::extendAlgae, m_algaeSubsystem).withTimeout(1.2).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    deAlgaeL3Trigger.onTrue(new RunCommand(m_algaeSubsystem::retractAlgae, m_algaeSubsystem).withTimeout(1.2).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    

    OverrideElevatorSafetyTrigger.onTrue(new InstantCommand(m_elevatorSubsystem::overrideElevatorSafety));

    coralManipulatingTrigger.onTrue(m_endEffectorSubsystem.coralToPosition());
    
    // Bongo
    L1BongoTrigger.onTrue(Commands.either(m_endEffectorSubsystem.runEndEffector(), m_elevatorSubsystem.moveElevatorTo("L1"), () -> m_elevatorSubsystem.isAtLevel("L1")));
    L2BongoTrigger.onTrue(Commands.either(m_endEffectorSubsystem.runEndEffector(), m_elevatorSubsystem.moveElevatorTo("L2"), () -> m_elevatorSubsystem.isAtLevel("L2")));
    L3BongoTrigger.onTrue(Commands.either(m_endEffectorSubsystem.runEndEffector(), m_elevatorSubsystem.moveElevatorTo("L3"), () -> m_elevatorSubsystem.isAtLevel("L3")));
    // L4BongoTrigger.onTrue(Commands.either(m_endEffectorSubsystem.runEndEffector(), m_elevatorSubsystem.moveElevatorTo("L4"), () -> m_elevatorSubsystem.isAtLevel("L4")));
    
    bongoPlaceL2Trigger.onTrue(Autos.coralPlacingAuto(m_elevatorSubsystem, m_endEffectorSubsystem, "L2"));
    bongoPlaceL3Trigger.onTrue(Autos.coralPlacingAuto(m_elevatorSubsystem, m_endEffectorSubsystem, "L3"));
    //bongoPlaceL4Trigger.onTrue(new CoralPlacingAuto(m_elevatorSubsystem, m_endEffectorSubsystem, 4).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    bongoZeroTrigger.onTrue(m_elevatorSubsystem.moveElevatorTo("ZERO"));
    clapIntakeTrigger.onTrue(new StartEndCommand(m_endEffectorSubsystem::runIntakeMaxSpeed, m_endEffectorSubsystem::stopIntake, m_endEffectorSubsystem).withTimeout(0.5).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    algaeToggleTrigger.onTrue(new StartEndCommand(m_algaeSubsystem::toggleAlgae, m_algaeSubsystem::stopAlgae, m_algaeSubsystem).withTimeout(0.5).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    // DDRL1Trigger.whileTrue(new RunCommand(() -> m_elevatorSubsystem.setElevatorPosition(ElevatorConstants.kElevatorSetpoints[0]), m_elevatorSubsystem));
    // DDRL2Trigger.whileTrue(new RunCommand(() -> m_elevatorSubsystem.setElevatorPosition(ElevatorConstants.kElevatorSetpoints[1]), m_elevatorSubsystem));
    // DDRL3Trigger.whileTrue(new RunCommand(() -> m_elevatorSubsystem.setElevatorPosition(ElevatorConstants.kElevatorSetpoints[2]), m_elevatorSubsystem));

    // DDRIntakeTrigger.onTrue(new RunCommand(m_endEffectorSubsystem::runIntakeMaxSpeed, m_endEffectorSubsystem).withTimeout(0.5));
    // DDRL4Trigger.whileTrue(new RunCommand(() -> m_elevatorSubsystem.setElevatorPosition(ElevatorConstants.kElevatorSetpoints[3]), m_elevatorSubsystem));
    DDRIncreaseSpeedTrigger.onTrue(new InstantCommand(this::addDDRSpeed));
    DDRDecreaseSpeedTrigger.onTrue(new InstantCommand(this::lowerDDRSpeed));

    balanceBoardCalibrate.onTrue(new InstantCommand(m_balanceBoard::calibrate));
  }

  public void updateDriveMode() {
    Command selected = m_driveChooser.getSelected();

    if (selected == null || selected.equals(activeDriveCommand)) {
        return;
    }

    // Cancel old command
    if (activeDriveCommand != null) {
        activeDriveCommand.cancel();
    }

    // Schedule new one
    selected.schedule();
    activeDriveCommand = selected;
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
    
    // return Autos.taxi(m_swerve);
    // return chaseTagCommand;
    
  }
  
}
