// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;

import static frc.robot.Constants.DriveConstants.*;

public class DriveTrain extends SubsystemBase {
  private SwerveDrivePoseEstimator poseEstimator;
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1); 
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);

  public DriveTrain() {
    m_gyro.reset();
    poseEstimator = new SwerveDrivePoseEstimator(
      kDriveKinematics,
      getGyroRotation(),
      getSwerveModulePositions(), 
      new Pose2d(),
      stateStdDevs,
      visionMeasurementStdDevs
    );

    RobotConfig config = new RobotConfig(
      ROBOT_MASS_KG,
      ROBOT_MOI, 
      new ModuleConfig(
        ModuleConstants.WHEEL_RADIUS,
        MAX_LINEAR_SPEED, 
        ModuleConstants.WHEEL_COF, 
        DCMotor.getKrakenX60Foc(1)
          .withReduction(ModuleConstants.DRIVE_GEAR_RATIO),
          ModuleConstants.CURRENT_LIMIT, 
          1),
      MODULE_TRANSLATIONS);
    AutoBuilder.configure(
      this::getPose, 
      this::setPose, 
      this::getRobotRelativeSpeeds, 
      (speeds, feedforwards) -> drive(speeds), 
      new PPHolonomicDriveController( 
              new PIDConstants(5.0, 0.0, 0.0), 
              new PIDConstants(5.0, 0.0, 0.0) 
      ),
      config, 
      () -> {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this 
    );
  }

  double speedMultiplier = 1;
  private Field2d field2d = new Field2d();
  
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
    }
  );

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
      m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      }
    );
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    return new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
    };
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kDriveKinematics.toChassisSpeeds(getSwerveModuleStates());
  };

  public SwerveModuleState[] getSwerveModuleStates() {
    return new SwerveModuleState[] {
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState()
    };
  }

  /*
   * Returns the currently-estimated pose of the robot from odometry
   * 
   * @return The pose
   */
  public Pose2d getOdometry() {
    return m_odometry.getPoseMeters();
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(getGyroRotation(), getSwerveModulePositions(), pose);
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
    speedMultiplier = 1.2;
    SmartDashboard.putNumber("Speed Multiplier", speedMultiplier);
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

    var swerveModuleStates = kDriveKinematics.toSwerveModuleStates(
        ChassisSpeeds.discretize(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot),
            kDrivePeriod));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, MAX_LINEAR_SPEED);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }
  public void drive(ChassisSpeeds speeds) {
    var swerveModuleStates = kDriveKinematics.toSwerveModuleStates(ChassisSpeeds.discretize(speeds, kDrivePeriod));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, MAX_LINEAR_SPEED);
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
   * Returns the heading of the robot from the gyro
   * @return the robot's heading as a rotation
   */
  public Rotation2d getGyroRotation() {
    return m_gyro.getRotation2d();
  }

  /*
   * Returns the rotation of the robot based on pose estimation
   */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }
  /**
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (kGyroReversed ? -1.0 : 1.0);
  }

  public void addVisionMeasurement(
      Pose2d visionMeasurementPose, 
      double timestamp, 
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(visionMeasurementPose, timestamp, visionMeasurementStdDevs);
  }

  public Field2d getPoseEstimatorField() {
    return field2d;
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Gyro Heading", getGyroRotation().getDegrees());
    updateOdometry();
    poseEstimator.update(
      getGyroRotation(), 
      getSwerveModulePositions());
    field2d.setRobotPose(getPose());
  }
}
