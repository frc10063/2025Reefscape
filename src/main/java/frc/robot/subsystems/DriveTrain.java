// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DriveConstants.*;

public class DriveTrain extends SubsystemBase {
  public DriveTrain() {
    m_gyro.reset();
  }

  double speedMultiplier = 1;
  private Field2d field2d = new Field2d();
  /** Creates a new DriveTrain. */
  private final SwerveModule m_frontLeft = new SwerveModule(
      kFrontLeftDriveMotorPort,
      kFrontLeftTurningMotorPort,
      kFrontLeftTurningEncoderPorts,
      kFrontLeftExpectedZero,
      kFrontLeftDriveEncoderReversed,
      kFrontLeftTurningEncoderReversed);
  private final SwerveModule m_rearLeft = new SwerveModule(
      kRearLeftDriveMotorPort,
      kRearLeftTurningMotorPort,
      kRearLeftTurningEncoderPorts,
      kRearLeftExpectedZero,
      kRearLeftDriveEncoderReversed,
      kRearLeftTurningEncoderReversed);
  private final SwerveModule m_rearRight = new SwerveModule(
      kRearRightDriveMotorPort,
      kRearRightTurningMotorPort,
      kRearRightTurningEncoderPorts,
      kRearRightExpectedZero,
      kRearRightDriveEncoderReversed,
      kRearRightTurningEncoderReversed);
  private final SwerveModule m_frontRight = new SwerveModule(
      kFrontRightDriveMotorPort,
      kFrontRightTurningMotorPort,
      kFrontRightTurningEncoderPorts,
      kFrontRightExpectedZero,
      kFrontRightDriveEncoderReversed,
      kFrontRightTurningEncoderReversed);

  private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      kDriveKinematics,
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

  public SwerveModulePosition[] getSwerveModulePositions() {
    return new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
    };
  }

  public SwerveModuleState[] getSwerveModuleStates() {
    return new SwerveModuleState[] {
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState()
    };
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

  public void slowSpeed() {
    speedMultiplier = 0.2;
    SmartDashboard.putNumber("Speed Multiplier", speedMultiplier);
  }

  public void defaultSpeed() {
    speedMultiplier = 1;
    SmartDashboard.putNumber("Speed Multiplier", speedMultiplier);
  }

  public void fastSpeed() {
    speedMultiplier = 1.167;
    SmartDashboard.putNumber("Speed Multiplier", speedMultiplier);
  }

  public void setSpeedMultiplier(double speedInput) {
    speedMultiplier = speedInput;
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    xSpeed = xSpeed * speedMultiplier;
    ySpeed = ySpeed * speedMultiplier;
    rot = rot * speedMultiplier;

    SmartDashboard.putNumber("XSpeed", xSpeed);
    SmartDashboard.putNumber("YSpeed", ySpeed);
    SmartDashboard.putNumber("Rotation", rot);
    SmartDashboard.putBoolean("Field Relative", fieldRelative);

    var maxSpeed = MAX_LINEAR_SPEED;

    var swerveModuleStates = kDriveKinematics.toSwerveModuleStates(
        ChassisSpeeds.discretize(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(ySpeed, xSpeed, rot),
            kDrivePeriod));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, maxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, MAX_LINEAR_SPEED);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
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

  public Rotation2d getGyroRotation() {
    return m_gyro.getRotation2d();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (kGyroReversed ? -1.0 : 1.0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Gyro Heading", getHeading());
    updateOdometry();
    field2d.setRobotPose(m_odometry.getPoseMeters());
    SmartDashboard.putData("Field", field2d);
    
  }
}
