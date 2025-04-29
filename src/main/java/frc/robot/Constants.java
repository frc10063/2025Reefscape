// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
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
    public static final int kRearRightDriveMotorPort = 4;
    public static final int kFrontRightDriveMotorPort = 5;

    public static final int kFrontLeftTurningMotorPort = 6;
    public static final int kRearLeftTurningMotorPort = 7;
    public static final int kRearRightTurningMotorPort = 8;
    public static final int kFrontRightTurningMotorPort = 9;

    public static final int kFrontLeftTurningEncoderPorts = 3;
    public static final int kRearLeftTurningEncoderPorts = 2;
    public static final int kRearRightTurningEncoderPorts = 1; 
    public static final int kFrontRightTurningEncoderPorts = 0;

    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kRearLeftTurningEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kRearRightTurningEncoderReversed = false;

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kRearLeftDriveEncoderReversed = false;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kRearRightDriveEncoderReversed = false;

    public static final double kFrontLeftExpectedZero = 0.935; // 0.935
    public static final double kRearLeftExpectedZero = 0.754; // 0.754
    public static final double kRearRightExpectedZero = 0.483; // 0.483
    public static final double kFrontRightExpectedZero = 0.436; // 0.436

    public static final double kDrivePeriod = TimedRobot.kDefaultPeriod;

    public static final Distance kTrackWidth = Meters.of(0.7);
    // Distance between centers of right and left wheels on robot

    public static final Distance kWheelBase = Meters.of(0.7);
    // Distance between front and back wheels on robot
    
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase.in(Meters) / 2, kTrackWidth.in(Meters) / 2),
            new Translation2d(kWheelBase.in(Meters) / 2, -kTrackWidth.in(Meters) / 2),
            new Translation2d(-kWheelBase.in(Meters) / 2, kTrackWidth.in(Meters) / 2),
            new Translation2d(-kWheelBase.in(Meters) / 2, -kTrackWidth.in(Meters) / 2));
    
    public static final boolean kGyroReversed = false;

    // The SysId tool provides a convenient method for obtaining these values for your robot.

    public static final LinearVelocity MAX_LINEAR_SPEED = MetersPerSecond.of(3.5);
    public static final LinearVelocity LINEAR_SPEED = MetersPerSecond.of(3);
    public static final AngularVelocity MAX_ANGULAR_VELOCITY = RadiansPerSecond.of(2 * Math.PI);
  }
  public static final class ModuleConstants {
    public static final AngularVelocity MAX_ANGULAR_VELOCITY = RadiansPerSecond.of(4 * Math.PI);
    public static final AngularAcceleration MAX_ANGULAR_ACCELERATION = RadiansPerSecondPerSecond.of(40 * Math.PI);

    public static final int kturningEncoderCPR = 4096;
    public static final int kdriveEncoderCPR = 42; // 4096

    public static final double drivekS = 0.1; // default 1
    public static final double drivekV = 2.07; // default 0.8
    public static final double drivekA = 0.5; // default 0.15
    
    public static final double turningkS = 0.2;
    public static final double turningkV = 0.5;
    public static final double turningkA = 0;

    public static final double DRIVE_GEAR_RATIO = 6.75;
    public static final double STEER_GEAR_RATIO =  150.0 / 7.0;
    public static final Distance WHEEL_DIAMETER = Inches.of(4);

    public static final Distance kDriveEncoderDistancePerPulse = ((WHEEL_DIAMETER.times(Math.PI)).div(kdriveEncoderCPR)).div(DRIVE_GEAR_RATIO);
        // 6.75 is rotations of motor per wheel rotation
        //((kWheelDiameterMeters * Math.PI) / (double) (kdriveEncoderCPR)) / 6.75; // 6.75 divided by 1.3?
    public static final double kDriveVelocityConversionFactor = (kDriveEncoderDistancePerPulse.magnitude() * (double) kdriveEncoderCPR) / 60.0;

    public static final double kTurningEncoderDistancePerPulse =
        // Assumes the encoders are on a 1:1 reduction with the module shaft.
        (2 * Math.PI) / ((double) kturningEncoderCPR);

    public static double turningKp = 8; //12.5
    public static double turningKd = 0.28;
    public static double turningKi = 0.01;
    
    public static double driveKp = 0.5; //0.5 
    public static double driveKd = 0.01875; // 0.01875
    public static double driveKi = 0.01; //0.01
  }


  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = 2 * Math.PI;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    
    public static final double robotCenterToFrontDistance = 0.425;

    public static final double coralLeftOffset = -0.171;
    public static final double coralRightOffset = -0.425;

    public static final double translationkP = 3;
    public static final double translationkI = 0;
    public static final double translationkD = 0;

    public static final double thetakP = 2;
    public static final double thetakI = 0;
    public static final double thetakD = 0;
  }

  public static final class ElevatorConstants {
    public static final int[] kElevatorPorts = new int[] {10, 11};

    public static final int[] kElevatorEncoders = new int[] {9, 8};
    public static final int kElevatorEncoderRes = 2048;

    public static final SparkBaseConfig LEFTELEVATOR_CONFIG = new SparkMaxConfig()
        .inverted(false)
        .idleMode(IdleMode.kBrake);
    public static final SparkBaseConfig RIGHTELEVATOR_CONFIG = new SparkMaxConfig()
        .inverted(false)
        .idleMode(IdleMode.kBrake);
    
    public static final double GEAR_RATIO = 12.75;
    
    public static final Distance SPOOL_DIAMETER = Inches.of(1.25);
    public static final Distance BASE_PLATE_HEIGHT = Inches.of(1.7);
    public static final Distance END_EFFECTOR_HEIGHT = Inches.of(20.3).plus(BASE_PLATE_HEIGHT);
    public static final Distance STARTING_HEIGHT = Inches.of(41);




    public static final double kSpoolCircumference = SPOOL_DIAMETER.magnitude() * Math.PI;

    public static final Distance kElevatorDistancePerPulse = ((SPOOL_DIAMETER.times(Math.PI)).div((double) kElevatorEncoderRes)).div(GEAR_RATIO);
    public static final double kElevatorMaxPosition = 24000; //22000?
    public static final double[] kElevatorSetpoints = new double[] {1744, 7148, 14000, 22800}; 
    public static final double[] kElevatorDeAlgaeSetpoints = new double[] {9300, 15000};

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kG = 0.28;
    public static final double kA = 0;
    public static final double kV = 0;
    public static final double kS = 0;

    public static final double kMaxVelocity = 0.3;
    public static final double kMaxAcceleration = 0.3;
  }
  public static final class IntakeConstants {
    
    public static final int kIntakePort = 12;

    public static final SparkBaseConfig ENDEFFECTOR_CONFIG = new SparkMaxConfig()
        .inverted(true);
    public static double kMaxSpeed = 0.6;
  }


  public static final class AlgaeConstants {
    public static final int kAlgaePort = 14; 
    public static final SparkBaseConfig ALGAE_CONFIG = new SparkMaxConfig()
        .inverted(true);
    
    // comlpete guess, in inches
    public static final double kSpoolDiamater = 2.5;
    public static final double kSpoolCircumference = kSpoolDiamater * Math.PI;
    public static final double kAlgaeEncoderCPR = 42;
    public static final int[] kAlgaeSetpoints = new int[] {4, 26};
    public static double kMaxSpeed = 0.3;
  }


  public static final class VisionConstants {
    public static final AprilTagFieldLayout APRIL_TAGS_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    public static final Transform3d camPosition = new Transform3d(
      0, 0, 0, 
      new Rotation3d(0, 0, 0));
    public static final int[] tagIds = {
      1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
      12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22
    };
  }
}
