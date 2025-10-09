// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
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
import static frc.robot.Constants.ModuleConstants.*;

public class SwerveModule extends SubsystemBase {

  int drivePort, turnPort;

  private final TalonFX m_driveMotor;
  private final SparkMax m_turningMotor;

  // private final RelativeEncoder m_driveEncoder;
  private final AnalogEncoder m_turningEncoder;

  private SparkBaseConfig driveConfig;

  public static double driveKp = startingDriveKp;
  public static double driveKi = startingDriveKi;
  public static double driveKd = startingDriveKd;

  public static double turningKp = startingTurningKp;
  public static double turningKi = startingTurningKi;
  public static double turningKd = startingTurningKd;

  SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(drivekS, drivekV, drivekA); 
  SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(turningkS, turningkV, turningkA); 
    
  private PIDController m_drivePIDController = new PIDController(driveKp, driveKi, driveKd);

  private ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          turningKp,
          turningKi,
          turningKd,
          new TrapezoidProfile.Constraints(
              MAX_ANGULAR_VELOCITY, 
              MAX_ANGULAR_ACCELERATION));
    

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
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderChannel,
      double expectedEncoderZero,
      boolean driveEncoderReversed,
      boolean turningEncoderReversed) {
    drivePort = driveMotorChannel;
    turnPort = turningMotorChannel;
    m_driveMotor = new TalonFX(driveMotorChannel);
    m_turningMotor = new SparkMax(turningMotorChannel, MotorType.kBrushless);

    // m_driveEncoder = m_driveMotor.getEncoder(); 
    m_turningEncoder = new AnalogEncoder(turningEncoderChannel, 1, expectedEncoderZero);
    
    m_turningPIDController.setTolerance(Units.degreesToRadians(1));

    m_turningEncoder.setInverted(turningEncoderReversed);
    driveConfig = new SparkMaxConfig()
        .inverted(driveEncoderReversed)
        .idleMode(IdleMode.kBrake);
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
    SmartDashboard.putNumber("Encoder Turn "+turnPort, m_turningEncoder.get());

    return new SwerveModuleState(
        m_driveMotor.getVelocity().refresh().getValueAsDouble() * kDriveVelocityConversionFactor, 
        new Rotation2d(m_turningEncoder.get() * kturningEncoderCPR * kTurningEncoderDistancePerPulse));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // return new SwerveModulePosition(
    //   m_driveEncoder.getDistance(), new Rotation2d(m_turningEncoder.getDistance()));
    // with scale (m_driveEncoder.getPosition() * kdriveEncoderCPR * kDriveEncoderDistancePerPulse)
    
    return new SwerveModulePosition(
        m_driveMotor.getPosition().refresh().getValueAsDouble()  * kDrivePositionConversionFactor,
        new Rotation2d(m_turningEncoder.get() * kturningEncoderCPR * kTurningEncoderDistancePerPulse));
  }
  
  /**
   * Sets the desired state for the module.
   * @param desiredState Desired state with speed and angle.
   */
  // var encoderRotation = new Rotation2d(m_turningEncoder.getDistance());
  public void setDesiredState(SwerveModuleState desiredState) {
    if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }
    var encoderRotation = new Rotation2d(m_turningEncoder.get() * kturningEncoderCPR * kTurningEncoderDistancePerPulse);
    SmartDashboard.putNumber("Turn Encoder "+turnPort, m_turningEncoder.get() * kturningEncoderCPR * kTurningEncoderDistancePerPulse);

    // Optimize the reference state to avoid spinning further than 90 degrees
    desiredState.optimize(encoderRotation);
    
    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    desiredState.cosineScale(encoderRotation);

    // ADDED FEEDFORWARD PARAM
    double driveFeedforward = m_driveFeedforward.calculate(desiredState.speedMetersPerSecond);
    double turnFeedforward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    double driveOutput =
        m_drivePIDController.calculate(m_driveMotor.getVelocity().refresh().getValueAsDouble() * kDriveVelocityConversionFactor, desiredState.speedMetersPerSecond);
    SmartDashboard.putNumber("Current Velocity" + drivePort, m_driveMotor.getVelocity().refresh().getValueAsDouble() * kDriveVelocityConversionFactor);
    SmartDashboard.putNumber("Desired Velocity" + drivePort, desiredState.speedMetersPerSecond);

    double turnOutput = 
        m_turningPIDController.calculate(
          (m_turningEncoder.get() * kturningEncoderCPR * kTurningEncoderDistancePerPulse), 
          desiredState.angle.getRadians());

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
    // this does nothing
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

