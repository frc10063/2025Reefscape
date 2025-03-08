// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.TimedRobot;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
// we have to go thorugh these once our robot is done
public final class Constants {
  public static final class OperatorConstants {
    public static final int kXBoxControllerPort = 0;
    public static final int kJoystickControllerPort = 1;
  }
  public static final class DriveConstants {
    public static final int kFrontLeftDriveMotorPort = 2;
    public static final int kRearLeftDriveMotorPort = 3;
    public static final int kFrontRightDriveMotorPort = 5;
    public static final int kRearRightDriveMotorPort = 4;

    public static final int kFrontLeftTurningMotorPort = 6;
    public static final int kRearLeftTurningMotorPort = 7;
    public static final int kFrontRightTurningMotorPort = 9;
    public static final int kRearRightTurningMotorPort = 8;

    public static final int kFrontLeftTurningEncoderPorts = 3;
    public static final int kRearLeftTurningEncoderPorts = 2;
    public static final int kFrontRightTurningEncoderPorts = 0;
    public static final int kRearRightTurningEncoderPorts = 1; 

    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kRearLeftTurningEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kRearRightTurningEncoderReversed = false;

    // public static final int[] kFrontLeftDriveEncoderPorts = new int[] {8, 9};
    // public static final int[] kRearLeftDriveEncoderPorts = new int[] {10, 11};
    // public static final int[] kFrontRightDriveEncoderPorts = new int[] {12, 13};
    // public static final int[] kRearRightDriveEncoderPorts = new int[] {14, 15};

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kRearLeftDriveEncoderReversed = false;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kRearRightDriveEncoderReversed = false;

    public static final double kFrontLeftExpectedZero = 0.935;
    public static final double kRearLeftExpectedZero = 0.754;
    public static final double kRearRightExpectedZero = 0.483;
    public static final double kFrontRightExpectedZero = 0.436;

    public static final double kDrivePeriod = TimedRobot.kDefaultPeriod;

    public static final double kTrackWidth = 0.5;
    // Distance between centers of right and left wheels on robot
    // we need to measure this out
    public static final double kWheelBase = 0.5;
    // Distance between front and back wheels on robot
    // Also measure this out
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
    public static final boolean kGyroReversed = false;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for your robot.
    // Help pls
    public static final double ksVolts = 1;
    public static final double kvVoltSecondsPerMeter = 0.8;
    public static final double kaVoltSecondsSquaredPerMeter = 0.15;

    public static final double kMaxSpeedMetersPerSecond = 3;
  }
  public static final class ModuleConstants {
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 8 * Math.PI;

    public static final int kturningEncoderCPR = 4096;
    public static final int kdriveEncoderCPR = 4096; // 42

    public static final double kWheelDiameterMeters = 0.1016;
    public static final double kDriveEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kdriveEncoderCPR;

    public static final double kTurningEncoderDistancePerPulse =
        // Assumes the encoders are on a 1:1 reduction with the module shaft.
        (2 * Math.PI) / (double) kturningEncoderCPR;

    public static final double kPModuleTurningController = 1;

    public static final double kPModuleDriveController = 1;
    public static final double turningKp = 10.4; // 9.6
    public static final double turningKd = 0.27;
    public static final double turningKi = 0.01;
    
    public static final double driveKp = 0.34; // 0.36
    public static final double driveKd = 0;
    public static final double driveKi = 0;
  }
  // This was part of the example, not sure if we will be using these
  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 2;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = 2 * Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
  public static final class ElevatorConstants {
    public static final int[] kElevatorPorts = new int[] {10, 11};


    public static final int[] kElevatorEncoders1 = new int[] {9, 8};
    public static final int kElevatorEncoderRes = 2048;
    public static final SparkBaseConfig LEFTELEVATOR_CONFIG = new SparkMaxConfig()
        .inverted(true)
        .idleMode(IdleMode.kBrake);
    public static final SparkBaseConfig RIGHTELEVATOR_CONFIG = new SparkMaxConfig()
        .inverted(true)
        .idleMode(IdleMode.kBrake);
    // temp
    public static final double kSpoolDiameter = 1;
    public static final double kElevatorDistancePerPulse = (kSpoolDiameter * Math.PI)/ (double) kElevatorEncoderRes;

  }
  public static final class IntakeConstants {
    // public static final int[] kIntakePorts = new int[] {0}; // Temp values
    public static final int kIntakePort = 0; // changed to single port
  }
}
