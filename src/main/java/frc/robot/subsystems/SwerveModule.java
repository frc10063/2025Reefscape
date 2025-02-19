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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {

  private final SparkMax m_driveMotor;
  private final SparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final Encoder m_turningEncoder;
  private double turningKp = 1;
  private double turningKd = 0;
  private double turningKi = 0;
  
  private double driveKp = 1;
  private double driveKd = 0;
  private double driveKi = 0;

  private SparkBaseConfig driveConfig;
  
  // This creates a PIDController object passing through the kP, kI, and kD parameters
  // We need to change these values, starting with kP and kI, then kI
  private final PIDController m_drivePIDController = new PIDController(driveKp, driveKi, driveKd);


  // This creates a ProfiledPIDController object passing through the kP, kI, and kD parameters
  // hte kp, kd, ki values should be in constants, I am testing this with smartdhasboard getNumber functions
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          turningKp,
          turningKi,
          turningKd,
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
      int[] turningEncoderChannels,
      boolean driveEncoderReversed,
      boolean turningEncoderReversed) {
    m_driveMotor = new SparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new SparkMax(turningMotorChannel, MotorType.kBrushless);

    m_driveEncoder = m_driveMotor.getEncoder(); 
    m_turningEncoder = new Encoder(turningEncoderChannels[0], turningEncoderChannels[1]);

    // Distance per pulse is basically distance driven for each count of the encoder
    // 2*pi*radius is circumference of the wheel, divided by encoder resolution would
    // give distance per encoder count

    // m_driveEncoder.setDistancePerPulse(ModuleConstants.kDriveEncoderDistancePerPulse);
    // m_driveEncoder.setReverseDirection(driveEncoderReversed); 
    
    // 2pi represents 360 degrees in radians, then divided by resolution to give
    // radians per count of encoder
    m_turningEncoder.setDistancePerPulse(ModuleConstants.kTurningEncoderDistancePerPulse);
    m_turningEncoder.setReverseDirection(turningEncoderReversed);

    driveConfig.encoder
        .positionConversionFactor(ModuleConstants.kDriveEncoderDistancePerPulse * ModuleConstants.kdriveEncoderCPR) 
        .velocityConversionFactor((ModuleConstants.kDriveEncoderDistancePerPulse * ModuleConstants.kdriveEncoderCPR) / 60)
        .inverted(driveEncoderReversed);
    // driveConfig.inverted(driveEncoderReversed); 
    // Idk what these parameters are
    m_driveMotor.configure(driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
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
    return new SwerveModuleState(
        m_driveEncoder.getVelocity(), 
        new Rotation2d(m_turningEncoder.getDistance()));
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
        new Rotation2d(m_turningEncoder.getDistance()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    var encoderRotation = new Rotation2d(m_turningEncoder.getDistance());

    // Optimize the reference state to avoid spinning further than 90 degrees
    desiredState.optimize(encoderRotation);

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    desiredState.cosineScale(encoderRotation);

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(m_driveEncoder.getVelocity(), desiredState.speedMetersPerSecond);


    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(
            m_turningEncoder.getDistance(), desiredState.angle.getRadians());

    m_driveMotor.setVoltage(driveOutput);
    m_turningMotor.setVoltage(turnOutput);
  }
  public void resetEncoders() {
    m_driveEncoder.setPosition(0);
    m_turningEncoder.reset();
  }
  @Override
  public void periodic() {
    turningKp = SmartDashboard.getNumber("Turning Kp", 1);
    turningKi = SmartDashboard.getNumber("Turning Ki", 0);
    turningKd = SmartDashboard.getNumber("Turning Kd", 0);

    driveKp = SmartDashboard.getNumber("Drive Kp", 1);
    driveKi = SmartDashboard.getNumber("Drive Ki", 0);
    driveKd = SmartDashboard.getNumber("Drive Kd", 0);


    SmartDashboard.putNumber("Current Turning Kp", turningKp);
    SmartDashboard.putNumber("Current Turning Ki", turningKi);
    SmartDashboard.putNumber("Current Turning Kd", turningKd);

    SmartDashboard.putNumber("Current Drive Kp", driveKp);
    SmartDashboard.putNumber("Current Drive Ki", driveKi);
    SmartDashboard.putNumber("Current Drive Kd", driveKd);
  }
}

