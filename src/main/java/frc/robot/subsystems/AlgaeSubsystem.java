// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;

// import frc.robot.Constants.AlgaeConstants;

// public class AlgaeSubsystem extends SubsystemBase {
//   private final SparkMax m_algaeMotor;
//   private final double kMaxSpeed = 0.01; // super slow just to be safe
//   /** Creates a new AlgaeSubsystem. */
//   public AlgaeSubsystem() {
//     m_algaeMotor = new SparkMax(AlgaeConstants.kAlgaePort, MotorType.kBrushless);
//     m_algaeMotor.configure(AlgaeConstants.ENDEFFECTOR_CONFIG, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
//   }

//   public void runAlgaeMaxSpeed() {
//     runAlgae(kMaxSpeed);
//   }
//   public void reverseAlgaeMaxSpeed() {
//     runAlgae(-kMaxSpeed);
//   }

//   public void runAlgae(double algaeSpeed) {
//     m_algaeMotor.set(algaeSpeed);
//   }

//   public void stopAlgae() {
//     m_algaeMotor.set(0);
//   }
// }