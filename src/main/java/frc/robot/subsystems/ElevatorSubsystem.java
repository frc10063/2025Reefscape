// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;

// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.ElevatorFeedforward;
// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  
  private final SparkMax m_elevatorRightMotor;
  private final SparkMax m_elevatorLeftMotor;


  // intialize the encoder objects
  private final Encoder m_elevatorEncoder;
  private static PIDController m_pidController; // --> OLD (is now NEW!)
  private static ProfiledPIDController m_profiledPIDController = new ProfiledPIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, 
        new TrapezoidProfile.Constraints(ElevatorConstants.kMaxVelocity, ElevatorConstants.kMaxAcceleration));
        
  private static ElevatorFeedforward m_feedforward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);
  public static double[] prevFeedGains;
  public static double[] DBFeedGains;
  public static double[] prevConstraints;
  public static double[] DBConstraints;

  // PID Constants
  // private double kP = ElevatorConstants.kP; //0.0005
  // private double kI = ElevatorConstants.kI;
  // private double kD = ElevatorConstants.kD;
  private boolean elevatorSafety = true;


  // MotionProfiling Constants
  // private double kMaxVelocity = ElevatorConstants.kElevatorMaxVelocity; // 0.5
  // private double kMaxAcceleration = ElevatorConstants.kElevatorMaxAcceleration; //0.375


  // Feed Forward Constants
  // private double kS = ElevatorConstants.kS;
  // private double kG = ElevatorConstants.kG;
  // private double kV = ElevatorConstants.kV;
  // private double kA = ElevatorConstants.kA;
  
  



  public ElevatorSubsystem() {
    // define motor and encoder objects -- I'm not sure what encoders they are
    m_elevatorRightMotor = new SparkMax(ElevatorConstants.kElevatorPorts[1], MotorType.kBrushless);
    m_elevatorLeftMotor = new SparkMax(ElevatorConstants.kElevatorPorts[0], MotorType.kBrushless);
    m_elevatorEncoder = new Encoder(ElevatorConstants.kElevatorEncoders1[0], ElevatorConstants.kElevatorEncoders1[1]);
    m_elevatorEncoder.setReverseDirection(false);;
    // m_elevatorEncoder = new Encoder(ElevatorConstants.kElevatorEncoders1[0], ElevatorConstants.kElevatorEncoders1[1]);

    m_elevatorRightMotor.configure(ElevatorConstants.RIGHTELEVATOR_CONFIG, 
        ResetMode.kNoResetSafeParameters, 
        PersistMode.kPersistParameters);
    m_elevatorLeftMotor.configure(ElevatorConstants.LEFTELEVATOR_CONFIG, 
        ResetMode.kNoResetSafeParameters, 
        PersistMode.kPersistParameters);
    m_elevatorEncoder.reset();
    m_elevatorEncoder.setDistancePerPulse(ElevatorConstants.kElevatorDistancePerPulse); // idk if necessary

    m_pidController = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);


    SmartDashboard.putNumber("Elev kP", m_profiledPIDController.getP());
    SmartDashboard.putNumber("Elev kI", m_profiledPIDController.getI());
    SmartDashboard.putNumber("Elev kD", m_profiledPIDController.getD());

    SmartDashboard.putNumber("Elev kG", m_feedforward.getKg());
    SmartDashboard.putNumber("Elev kV", m_feedforward.getKv());
    SmartDashboard.putNumber("Elev kA", m_feedforward.getKa());
    SmartDashboard.putNumber("Elev kS", m_feedforward.getKs());

    SmartDashboard.putNumber("Elev Max Velocity", m_profiledPIDController.getConstraints().maxVelocity);
    SmartDashboard.putNumber("Elev Max Accel", m_profiledPIDController.getConstraints().maxAcceleration);
    SmartDashboard.putNumber("Elev kP", m_profiledPIDController.getP());

    prevFeedGains = new double[] {m_feedforward.getKs(), m_feedforward.getKg(), m_feedforward.getKv(), m_feedforward.getKa()};
    DBFeedGains = new double[] {
      SmartDashboard.getNumber("Elev kS", m_feedforward.getKs()),
      SmartDashboard.getNumber("Elev kG", m_feedforward.getKg()),
      SmartDashboard.getNumber("Elev kV", m_feedforward.getKv()),
      SmartDashboard.getNumber("Elev kA", m_feedforward.getKa())};
  }
      
  public void moveElevator(double elevatorSpeed) {
    double encoderValue = m_elevatorEncoder.get();
    double maxPosition = ElevatorConstants.kElevatorMaxPosition;
    if (elevatorSpeed == 0) {
      m_elevatorLeftMotor.setVoltage(ElevatorConstants.kG);
      m_elevatorRightMotor.setVoltage(ElevatorConstants.kG);
    } else {
      if (elevatorSafety == true) {
        if ((encoderValue < 0 && elevatorSpeed < 0) || (encoderValue >= maxPosition && elevatorSpeed > 0)) {
          m_elevatorRightMotor.set(0);
          m_elevatorLeftMotor.set(0);
        } else {
          m_elevatorRightMotor.set(elevatorSpeed);
          m_elevatorLeftMotor.set(elevatorSpeed);
        }
      } else {
        m_elevatorRightMotor.set(elevatorSpeed);
        m_elevatorLeftMotor.set(elevatorSpeed);
      }
    } 
  }

  public void overrideElevatorSafety() {
    elevatorSafety = !elevatorSafety;
  }


  public void setElevatorPosition(double targetPosition) {
    double currentPosition = m_elevatorEncoder.get();
    // double pidOutput = m_pidController.calculate(currentPosition, targetPosition); // changed m_pidController to profile
    double pidOutput = m_profiledPIDController.calculate(currentPosition, targetPosition);
    double feedforwardTerm = m_feedforward.calculate(m_profiledPIDController.getSetpoint().velocity);
    double output = pidOutput + feedforwardTerm;

    // moveElevator(output);
    m_elevatorLeftMotor.setVoltage(output);
    m_elevatorRightMotor.setVoltage(output);

    SmartDashboard.putNumber("PID Output", pidOutput);
    SmartDashboard.putNumber("Feed Foward", feedforwardTerm);
    SmartDashboard.putNumber("Elevator Desired Pos", targetPosition);
  }
  public void stop() {
    m_elevatorLeftMotor.setVoltage(ElevatorConstants.kG);
    m_elevatorRightMotor.setVoltage(ElevatorConstants.kG);
  }


  public void SetGoal(double targetPosition) {
    // resets start position profiledPID to current position 
    // m_profiledPIDController.reset(m_elevatorEncoder.get());
    // m_profiledPIDController.setGoal(targetPosition);
  }


  @Override
  public void periodic() {
    
    SmartDashboard.putNumber("Elevator Speed", m_elevatorLeftMotor.get());
    SmartDashboard.putNumber("Position", m_elevatorEncoder.get());

    m_profiledPIDController.setP(SmartDashboard.getNumber("Elev kP", m_profiledPIDController.getP()));
    m_profiledPIDController.setI(SmartDashboard.getNumber("Elev kI", m_profiledPIDController.getI()));
    m_profiledPIDController.setD(SmartDashboard.getNumber("Elev kD", m_profiledPIDController.getD()));
    
    DBFeedGains = new double[] {
      SmartDashboard.getNumber("Elev kS", m_feedforward.getKs()),
      SmartDashboard.getNumber("Elev kG", m_feedforward.getKg()),
      SmartDashboard.getNumber("Elev kV", m_feedforward.getKv()),
      SmartDashboard.getNumber("Elev kA", m_feedforward.getKa())
    };
    DBConstraints = new double[] {
      SmartDashboard.getNumber("Elev Max Velocity", m_profiledPIDController.getConstraints().maxVelocity),
      SmartDashboard.getNumber("Elev Max Accel", m_profiledPIDController.getConstraints().maxAcceleration)
    };
    if (DBFeedGains != prevFeedGains) {
      m_feedforward = new ElevatorFeedforward(DBFeedGains[0], DBFeedGains[1], DBFeedGains[2], DBFeedGains[3]);
      prevFeedGains = DBFeedGains;
    }
    if (DBConstraints != prevConstraints) {
      m_profiledPIDController.setConstraints(new TrapezoidProfile.Constraints(DBConstraints[0], DBConstraints[1]));
      prevConstraints = DBConstraints;
    }
  }
}
