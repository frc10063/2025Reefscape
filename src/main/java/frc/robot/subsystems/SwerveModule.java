// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {

  private final PWMSparkMax m_driveMotor;
  private final PWMSparkMax m_turningMotor;

  private final Encoder m_driveEncoder;
  private final Encoder m_turningEncoder;
  private double turningKp = 1;
  private double turningKd = 0;
  private double turningKi = 0;

  // This creates a PIDController object passing through the kP, kI, and kD parameters
  // We need to change these values, starting with kP and kI, then kI
  private final PIDController m_drivePIDController = new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);


  // This creates a ProfiledPIDController object passing through the kP, kI, and kD parameters
  // hte kp, kd, ki values should be in constants, I am testing this with smartdhasboard getNumber functions
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          turningKp,
          turningKd,
          turningKi,
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
      int[] driveEncoderChannels,
      int[] turningEncoderChannels,
      boolean driveEncoderReversed,
      boolean turningEncoderReversed) {
    m_driveMotor = new PWMSparkMax(driveMotorChannel);
    m_turningMotor = new PWMSparkMax(turningMotorChannel);

    m_driveEncoder = new Encoder(driveEncoderChannels[0], driveEncoderChannels[1]);
    m_turningEncoder = new Encoder(turningEncoderChannels[0], turningEncoderChannels[1]);

    // Distance per pulse is basically distance driven for each count of the encoder
    // 2*pi*radius is circumference of the wheel, divided by encoder resolution would
    // give distance per encoder count
    m_driveEncoder.setDistancePerPulse(ModuleConstants.kTurningEncoderDistancePerPulse);

    m_driveEncoder.setReverseDirection(driveEncoderReversed);
    // 2pi represents 360 degrees in radians, then divided by resolution to give
    // radians per count of encoder
    m_turningEncoder.setDistancePerPulse(ModuleConstants.kTurningEncoderDistancePerPulse);
    
    m_turningEncoder.setReverseDirection(turningEncoderReversed);
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
    return new SwerveModuleState(
        m_driveEncoder.getRate(), new Rotation2d(m_turningEncoder.getDistance()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getDistance(), new Rotation2d(m_turningEncoder.getDistance()));
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
        m_drivePIDController.calculate(m_driveEncoder.getRate(), desiredState.speedMetersPerSecond);


    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(
            m_turningEncoder.getDistance(), desiredState.angle.getRadians());

    m_driveMotor.setVoltage(driveOutput);
    m_turningMotor.setVoltage(turnOutput);
  }
  public void resetEncoders() {
    m_driveEncoder.reset();
    m_turningEncoder.reset();
  }
  @Override
  public void periodic() {
    turningKp = SmartDashboard.getNumber("Turning Kp", 1);
    turningKd = SmartDashboard.getNumber("Turning Kd", 0);
    turningKi = SmartDashboard.getNumber("Turning Ki", 0);

    SmartDashboard.putNumber("Current Kp", turningKp);
    SmartDashboard.putNumber("Current Kd", turningKd);
    SmartDashboard.putNumber("Current Ki", turningKi);
  }
}

