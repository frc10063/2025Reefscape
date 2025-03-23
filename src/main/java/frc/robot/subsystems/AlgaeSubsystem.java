// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;

public class AlgaeSubsystem extends SubsystemBase {
  private final SparkMax m_algaeMotor;
  private final RelativeEncoder m_algaeEncoder;
  private final double maxSpeed = AlgaeConstants.kMaxSpeed; // super slow just to be safe
  /** Creates a new AlgaeSubsystem. */
  public AlgaeSubsystem() {
    m_algaeMotor = new SparkMax(AlgaeConstants.kAlgaePort, MotorType.kBrushless);
    m_algaeMotor.configure(AlgaeConstants.ALGAE_CONFIG, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_algaeEncoder = m_algaeMotor.getEncoder();
    m_algaeEncoder.setPosition(0);
  }

  public void runAlgaeMaxSpeed() {
    if (m_algaeEncoder.getPosition() > AlgaeConstants.kAlgaeSetpoints[1]) {
      stopAlgae();
    } else {
      runAlgae(maxSpeed);
    }
    
  }
  public void reverseAlgaeMaxSpeed() {
    if (m_algaeEncoder.getPosition() < AlgaeConstants.kAlgaeSetpoints[0]) {
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
  }
  public void retractAlgae() {
    while (m_algaeEncoder.getPosition() > AlgaeConstants.kAlgaeSetpoints[0]) {
      reverseAlgaeMaxSpeed();
    }
    stopAlgae();
  }

  public void stopAlgae() {
    m_algaeMotor.set(0);
  }
  
}