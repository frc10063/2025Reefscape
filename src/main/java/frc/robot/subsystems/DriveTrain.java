// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

import com.studica.frc.AHRS;

public class DriveTrain extends SubsystemBase {
  public DriveTrain() {
    m_gyro.reset();
  }
  public double speedMultiplier = 1;
  /** Creates a new DriveTrain. */
  private final SwerveModule m_frontLeft = 
      new SwerveModule(
        DriveConstants.kFrontLeftDriveMotorPort, 
        DriveConstants.kFrontLeftTurningMotorPort,
        DriveConstants.kFrontLeftTurningEncoderPorts,
        DriveConstants.kFrontLeftExpectedZero,
        DriveConstants.kFrontLeftDriveEncoderReversed,
        DriveConstants.kFrontLeftTurningEncoderReversed);
  private final SwerveModule m_rearLeft = 
      new SwerveModule(
        DriveConstants.kRearLeftDriveMotorPort, 
        DriveConstants.kRearLeftTurningMotorPort,
        DriveConstants.kRearLeftTurningEncoderPorts,
        DriveConstants.kRearLeftExpectedZero, 
        DriveConstants.kRearLeftDriveEncoderReversed,
        DriveConstants.kRearLeftTurningEncoderReversed);
  private final SwerveModule m_rearRight =
      new SwerveModule(
        DriveConstants.kRearRightDriveMotorPort, 
        DriveConstants.kRearRightTurningMotorPort,
        DriveConstants.kRearRightTurningEncoderPorts,
        DriveConstants.kRearRightExpectedZero, 
        DriveConstants.kRearRightDriveEncoderReversed,
        DriveConstants.kRearRightTurningEncoderReversed);
  private final SwerveModule m_frontRight = 
      new SwerveModule(
        DriveConstants.kFrontRightDriveMotorPort, 
        DriveConstants.kFrontRightTurningMotorPort,
        DriveConstants.kFrontRightTurningEncoderPorts,
        DriveConstants.kFrontRightExpectedZero,
        DriveConstants.kFrontRightDriveEncoderReversed,
        DriveConstants.kFrontRightTurningEncoderReversed);



  private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          DriveConstants.kDriveKinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
          });
  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });
      }
  /*
   * Returns the currently-estimated pose of the robot
   * 
   * @return The pose
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
      m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
      },
      pose);
  }
  public void setHalfSpeed() {
    speedMultiplier = 0.5;
  }
  public void setDefaultSpeed() {
    speedMultiplier = 1;
  }
  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    SmartDashboard.putNumber("XSpeed", xSpeed * speedMultiplier);
    SmartDashboard.putNumber("YSpeed", ySpeed * speedMultiplier);
    SmartDashboard.putNumber("Rotation", rot * speedMultiplier);
    xSpeed = xSpeed * speedMultiplier;
    ySpeed = ySpeed * speedMultiplier;
        var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot),
                DriveConstants.kDrivePeriod));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }
  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
  @Override
  public void periodic() {
    
  }
}

