// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {

  int drivePort, turnPort;

  private final SparkMax m_driveMotor;
  private final SparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final AnalogEncoder m_turningEncoder;

  private SparkBaseConfig driveConfig;

  public static double driveKp = ModuleConstants.driveKp;
  public static double driveKi = ModuleConstants.driveKi;
  public static double driveKd = ModuleConstants.driveKd;

  public static double turningKp = ModuleConstants.turningKp;
  public static double turningKi = ModuleConstants.turningKi;
  public static double turningKd = ModuleConstants.turningKd;

  SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(ModuleConstants.drivekS, ModuleConstants.drivekV, ModuleConstants.drivekA); 
  SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(ModuleConstants.turningkS, ModuleConstants.turningkV, ModuleConstants.turningkA); 
    
    
  // This creates a PIDController object passing through the kP, kI, and kD parameters
  // We need to change these values, starting with kP and kI, then kI
  private PIDController m_drivePIDController = new PIDController(ModuleConstants.driveKp, ModuleConstants.driveKi, ModuleConstants.driveKd);
  
  // private final PIDController m_turningPIDController = new PIDController(turningKp, turningKi, turningKd);

  // This creates a ProfiledPIDController object passing through the kP, kI, and kD parameters
  // hte kp, kd, ki values should be in constants, I am testing this with smartdhasboard getNumber functions
  private ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          /*ModuleConstants.*/turningKp,
          /*ModuleConstants.*/turningKi,
          /*ModuleConstants.*/turningKd,
          new TrapezoidProfile.Constraints(
              ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond, 
              ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));
    

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorChannel PWM output for the drive motor.
   * @param turningMotorChannel PWM output for the turning motor.
   * @param driveEncoderChannelA DIO input for the drive encoder channel A
   * @param driveEncoderChannelB DIO input for the drive encoder channel B
   * @param turningEncoderChannelA DIO input for the turning encoder channel A
   * @param turningEncoderChannelB DIO input for the turning encoder channel B
   */

  // This is the constructor for the SwerveModule class
  // When creating an instance of the class (in DriveTrain), you will put in these parameters
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderChannel,
      double expectedEncoderZero,
      boolean driveEncoderReversed,
      boolean turningEncoderReversed) {
    drivePort = driveMotorChannel;
    turnPort = turningMotorChannel;
    m_driveMotor = new SparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new SparkMax(turningMotorChannel, MotorType.kBrushless);

    m_driveEncoder = m_driveMotor.getEncoder(); 
    m_turningEncoder = new AnalogEncoder(turningEncoderChannel, 1, expectedEncoderZero);
    
    m_turningPIDController.setTolerance(Units.degreesToRadians(1));
    // Distance per pulse is basically distance driven for each count of the encoder
    // 2*pi*radius is circumference of the wheel, divided by encoder resolution would
    // give distance per encoder count

    // m_driveEncoder.setDistancePerPulse(ModuleConstants.kDriveEncoderDistancePerPulse);
    // m_driveEncoder.setReverseDirection(driveEncoderReversed); 
    
    // 2pi represents 360 degrees in radians, then divided by resolution to give
    // radians per count of encoder

    // this method doesnt exist for analogencoders
    //m_turningEncoder.setDistancePerPulse(ModuleConstants.kTurningEncoderDistancePerPulse);
    m_turningEncoder.setInverted(turningEncoderReversed);
    driveConfig = new SparkMaxConfig()
        .inverted(driveEncoderReversed)
        .idleMode(IdleMode.kBrake);
    driveConfig.encoder
        .positionConversionFactor(ModuleConstants.kDriveEncoderDistancePerPulse * ModuleConstants.kdriveEncoderCPR) 
        .velocityConversionFactor(ModuleConstants.kDriveVelocityConversionFactor);
        // (ModuleConstants.kDriveEncoderDistancePerPulse * (double) ModuleConstants.kdriveEncoderCPR) / 60.0
    
        // .inverted(driveEncoderReversed);
    // driveConfig.inverted(driveEncoderReversed); 
    // Idk what these parameters are
    m_driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous. 
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }
  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // return new SwerveModuleState(
    //     m_driveEncoder.getRate(), new Rotation2d(m_turningEncoder.getDistance()));
    // with scale (m_driveEncoder.getVelocity() * ModuleConstants.kdriveEncoderCPR * ModuleConstants.kDriveEncoderDistancePerPulse)/60

    SmartDashboard.putNumber("Encoder Turn "+turnPort, m_turningEncoder.get());
    return new SwerveModuleState(
        m_driveEncoder.getVelocity(), 
        new Rotation2d(m_turningEncoder.get() * ModuleConstants.kturningEncoderCPR * ModuleConstants.kTurningEncoderDistancePerPulse));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // return new SwerveModulePosition(
    //   m_driveEncoder.getDistance(), new Rotation2d(m_turningEncoder.getDistance()));
    // with scale (m_driveEncoder.getPosition() * ModuleConstants.kdriveEncoderCPR * ModuleConstants.kDriveEncoderDistancePerPulse)
    
    return new SwerveModulePosition(
        m_driveEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.get() * ModuleConstants.kturningEncoderCPR * ModuleConstants.kTurningEncoderDistancePerPulse));
  }
  
  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  // var encoderRotation = new Rotation2d(m_turningEncoder.getDistance());
  public void setDesiredState(SwerveModuleState desiredState) {
    if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }
    var encoderRotation = new Rotation2d(m_turningEncoder.get() * ModuleConstants.kturningEncoderCPR * ModuleConstants.kTurningEncoderDistancePerPulse);
    SmartDashboard.putNumber("Turn Encoder "+turnPort, m_turningEncoder.get() * ModuleConstants.kturningEncoderCPR * ModuleConstants.kTurningEncoderDistancePerPulse);
    // Optimize the reference state to avoid spinning further than 90 degrees
    desiredState.optimize(encoderRotation);
    
    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    desiredState.cosineScale(encoderRotation);
    final double driveFeedforward = m_driveFeedforward.calculate(desiredState.speedMetersPerSecond);
    final double turnFeedforward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);
    // Calculate the drive output from the drive PID controller.
    double driveOutput =
        m_drivePIDController.calculate(m_driveEncoder.getVelocity(), desiredState.speedMetersPerSecond);
    SmartDashboard.putNumber("Current Velocity" + drivePort, m_driveEncoder.getVelocity());
    SmartDashboard.putNumber("Desired Velocity" + drivePort, desiredState.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    // final double turnOutput =
    //     m_turningPIDController.calculate(
    //         m_turningEncoder.getDistance(), desiredState.angle.getRadians());
    double turnOutput = 
        m_turningPIDController.calculate(
          (m_turningEncoder.get() * ModuleConstants.kturningEncoderCPR * ModuleConstants.kTurningEncoderDistancePerPulse), 
          desiredState.angle.getRadians());

    // driveOutput = (driveOutput) * speedMultiplier;
    driveOutput = (driveOutput + driveFeedforward);
    turnOutput = -(turnOutput);

    SmartDashboard.putNumber("Desired angle" + turnPort, desiredState.angle.getRadians());
    SmartDashboard.putNumber("Drive Motor "+drivePort, driveOutput);
    SmartDashboard.putNumber("Turn Motor "+turnPort, turnOutput);
    m_driveMotor.setVoltage(driveOutput);
    m_turningMotor.setVoltage(turnOutput);
  }
  public void stop() {
    m_driveMotor.set(0);
    m_turningMotor.set(0);
  }
  public void resetEncoders() {
    m_driveEncoder.setPosition(0);
    // m_turningEncoder.reset();
  }
  
  public static void putPIDDashboard() {
    SmartDashboard.putNumber("Turning Kp", turningKp);
    SmartDashboard.putNumber("Turning Ki", turningKi);
    SmartDashboard.putNumber("Turning Kd", turningKd);

    SmartDashboard.putNumber("Drive Kp", driveKp);
    SmartDashboard.putNumber("Drive Ki", driveKi);
    SmartDashboard.putNumber("Drive Kd", driveKd);
  }
  public static void getPIDDashboard() { 
    turningKp = SmartDashboard.getNumber("Turning Kp", turningKp);
    turningKi = SmartDashboard.getNumber("Turning Ki", turningKi);
    turningKd = SmartDashboard.getNumber("Turning Kd", turningKd);

    driveKp = SmartDashboard.getNumber("Drive Kp", driveKp);
    driveKi = SmartDashboard.getNumber("Drive Ki", driveKi);
    driveKd = SmartDashboard.getNumber("Drive Kd", driveKd);
  }
  @Override
  public void periodic() {
    getPIDDashboard();
    
    m_turningPIDController.setPID(turningKp, turningKi, turningKd);
    m_drivePIDController.setPID(driveKp, driveKi, driveKd);
  }
  }

