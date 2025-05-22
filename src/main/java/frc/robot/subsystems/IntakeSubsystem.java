package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    // private final SparkMax m_intakeRightMotor;
    // private final SparkMax m_intakeLeftMotor;
    private final SparkMax m_intakeMotor;
    ElevatorSubsystem m_elevatorSubsystem;

    private final double kMaxSpeed = 0.4; // Full speed? Not sure if it's needed
    // public double L4SpeedMultiplier = 1;

    public IntakeSubsystem(ElevatorSubsystem m_elevatorSubsystem) {
        this.m_elevatorSubsystem = m_elevatorSubsystem;
        m_intakeMotor = new SparkMax(IntakeConstants.kIntakePort, MotorType.kBrushless);
        m_intakeMotor.configure(IntakeConstants.ENDEFFECTOR_CONFIG, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        // invert to spin in opposite directions prob
        // m_intakeLeftMotor.setInverted(true);
    }

    public Command runIntakeMaxSpeed() {
        return Commands.run(() -> runIntake(kMaxSpeed)).withTimeout(1).finallyDo(this::stopIntake);
    }

    public void runIntake(double intakeSpeed) {
        m_intakeMotor.set(intakeSpeed);
    }

    public void stopIntake() {
        m_intakeMotor.set(0);
    }

    public Command runL1EndEffector() {
        return Commands.run(() -> this.runIntake(0.2)).withTimeout(2).finallyDo(this::stopIntake);
    }
    public Command runEndEffector() {
        return Commands.either(runL1EndEffector(), runIntakeMaxSpeed(), () -> m_elevatorSubsystem.isAtLevel("L1"));
    }
}