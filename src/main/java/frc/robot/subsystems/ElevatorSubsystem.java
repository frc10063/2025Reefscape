// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.ElevatorFeedfoward;
// import edu.wpi.first.math.controller.PIDController;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  private final SparkMax m_elevatorRightMotor;
  private final SparkMax m_elevatorLeftMotor;

  // intialize the encoder objects
  private final Encoder m_elevatorEncoder;
  private final ElevatorFeedforward m_feedfoward;
  // private final PIDController m_pidController; --> OLD
  private final ProfiledPIDController m_profiledPIDController; // new
  
  // PID Constants
  private final double kP = 1;
  private final double kI = 0;
  private final double kD = 0;

  // MotionProfiling Constants
  private final double kMaxVelocity = 0.3;
  private final double kMaxAcceleration = 0.3;

  // Feed Foward Constants
  private final double kS = 0;
  private final double kG = 0;
  private final double kV = 0;


  public ElevatorSubsystem() {
    // define motor and encoder objects -- I'm not sure what encoders they are
    m_elevatorRightMotor = new SparkMax(ElevatorConstants.kElevatorPorts[1], MotorType.kBrushless);
    m_elevatorLeftMotor = new SparkMax(ElevatorConstants.kElevatorPorts[0], MotorType.kBrushless);
    m_elevatorEncoder = new Encoder(ElevatorConstants.kElevatorEncoders1[0], ElevatorConstants.kElevatorEncoders1[1]);

    m_elevatorRightMotor.configure(ElevatorConstants.RIGHTELEVATOR_CONFIG, 
        ResetMode.kNoResetSafeParameters, 
        PersistMode.kPersistParameters);
    m_elevatorLeftMotor.configure(ElevatorConstants.LEFTELEVATOR_CONFIG, 
        ResetMode.kNoResetSafeParameters, 
        PersistMode.kPersistParameters);

    m_elevatorEncoder.setDistancePerPulse(ElevatorConstantsConstants.kEncoderDistancePerPulse); // idk if necessary

    // m_pidController = new PIDController(kP, kI, kD);

    m_profiledPIDController = new ProfiledPIDController(kP, kI, kD, 
        new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration));
        m_feedforward = new ElevatorFeedforward(kS, kG, kV);
  }

  public void moveElevator(double elevatorSpeed) {
    m_elevatorLeftMotor.set(elevatorSpeed);
    m_elevatorRightMotor.set(elevatorSpeed);
  }
  
  public void setElevatorPosition(double targetPosition) {
    double currentPosition = m_elevatorEncoder.get();
    double pidOutput = m_profiledPIDController.calculate(currentPosition, targetPosition); // changed m_pidController to profile
    double feedforwardTerm = m_feedforward.calculate(m_profiledPIDController.getSetpoint().velocity);
    double output = pidOutput + feedforwardTerm;

    m_elevatorLeftMotor.set(output);
    m_elevatorRightMotor.set(output);
  }

  public void SetGoal(double targetPosition) {
    // resets start position profiledPID to current position 
    m_profiledPIDController.reset(m_elevatorEncoder.get());
    m_profiledPIDController.setGoal(targetPosition);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Speed", m_elevatorLeftMotor.get());
    SmartDashboard.putNumber("Position", m_elevatorEncoder.get());
  }
}
