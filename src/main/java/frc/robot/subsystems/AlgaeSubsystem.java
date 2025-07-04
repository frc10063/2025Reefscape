// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;

public class AlgaeSubsystem extends SubsystemBase {
  private final SparkMax m_algaeMotor;
  private final RelativeEncoder m_algaeEncoder;
  private boolean isExtended = false;
  private final double maxSpeed = AlgaeConstants.kMaxSpeed; 
  /** Creates a new AlgaeSubsystem. */
  public AlgaeSubsystem() {
    m_algaeMotor = new SparkMax(AlgaeConstants.kAlgaePort, MotorType.kBrushless);
    m_algaeMotor.configure(AlgaeConstants.ALGAE_CONFIG, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_algaeEncoder = m_algaeMotor.getEncoder();
    m_algaeEncoder.setPosition(0);
  }

  public void runAlgaeMaxSpeed() {
    if (m_algaeEncoder.getPosition() >= AlgaeConstants.kAlgaeSetpoints[1]) {
      stopAlgae();
    } else {
      runAlgae(maxSpeed);
    }
    
  }
  public void reverseAlgaeMaxSpeed() {
    if (m_algaeEncoder.getPosition() <= AlgaeConstants.kAlgaeSetpoints[0]) {
      stopAlgae();
    } else {
      runAlgae(-maxSpeed);
    }
  }

  public void runAlgae(double algaeSpeed) {
    m_algaeMotor.set(algaeSpeed);
  }
  public void extendAlgae() {
    while (m_algaeEncoder.getPosition() < AlgaeConstants.kAlgaeSetpoints[1]) {
      runAlgaeMaxSpeed();
    }
    stopAlgae();
    isExtended = true;
    
  }
  public void retractAlgae() {
    while (m_algaeEncoder.getPosition() > AlgaeConstants.kAlgaeSetpoints[0]) {
      reverseAlgaeMaxSpeed();
    }
    stopAlgae();
    isExtended = false;
  }
  public boolean algaeIsExtended() {
    return isExtended;
  }
  public void stopAlgae() {
    m_algaeMotor.set(0);
  }
  public void toggleAlgae() {
    if (isExtended) {
      retractAlgae();
    } else {
      extendAlgae();
    }
  }


  public Command extendAlgaeCommand() {
    return Commands.run(this::extendAlgae).withTimeout(2);
  }
  public Command retractAlgaeCommand() {
    return Commands.run(this::retractAlgae).withTimeout(2);
  }
  // dlnvm ziv hxzib
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Algae Position", m_algaeEncoder.getPosition());
    SmartDashboard.putNumber("Algae Motor", m_algaeMotor.get());
    SmartDashboard.putBoolean("Algae Is Extended?", isExtended);
  }
  
}