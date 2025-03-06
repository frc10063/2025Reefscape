// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;

// import frc.robot.Constants.IntakeConstants;

// public class IntakeSubsystem extends SubsystemBase {

//     private final SparkMax m_intakeRightMotor;
//     private final SparkMax m_intakeLeftMotor;
//     private final double kMaxSpeed = 1.0; // Full speed? Not sure if it's needed

//     public IntakeSubsystem() {
//         m_intakeRightMotor = new SparkMax(IntakeConstants.kIntakePorts[1], MotorType.kBrushless);
//         m_intakeLeftMotor = new SparkMax(IntakeConstants.kIntakePorts[0], MotorType.kBrushless);

//         // invert to spin in opposite directions prob
//         m_intakeLeftMotor.setInverted(true);
//     }

//     public void runIntakeMaxSpeed() {
//         runIntake(kMaxSpeed);
//     }

//     public void runIntake(double intakeSpeed) {
//         m_intakeRightMotor.set(intakeSpeed);
//         m_intakeLeftMotor.set(intakeSpeed);
//     }

//     public void stopIntake() {
//         m_intakeRightMotor.set(0);
//         m_intakeLeftMotor.set(0);
//     }
// }