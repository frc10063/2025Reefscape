package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static frc.robot.Constants.EndEffectorConstants.*;

public class EndEffectorSubsystem extends SubsystemBase {

    private final SparkMax m_endEffectorMotor;
    ElevatorSubsystem m_elevatorSubsystem;
    

    public EndEffectorSubsystem(ElevatorSubsystem m_elevatorSubsystem) {
        this.m_elevatorSubsystem = m_elevatorSubsystem;
        m_endEffectorMotor = new SparkMax(END_EFFECTOR_PORT, MotorType.kBrushless);
        m_endEffectorMotor.configure(ENDEFFECTOR_CONFIG, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void runIntake(double intakeSpeed) {
        m_endEffectorMotor.set(intakeSpeed);
    }

    public void stopIntake() {
        m_endEffectorMotor.set(0);
    }

    public Command runIntakeMaxSpeed() {
        return Commands.run(() -> runIntake(MAX_SPEED)).withTimeout(1).finallyDo(this::stopIntake);
    }

    public Command runL1EndEffector() {
        return Commands.run(() -> this.runIntake(0.2)).withTimeout(2).finallyDo(this::stopIntake);
    }
    public Command runEndEffector() {
        return Commands.either(runL1EndEffector(), runIntakeMaxSpeed(), () -> m_elevatorSubsystem.isAtLevel("L1"));
    }
    public Command coralToPosition() {
        return Commands.run(() -> this.runIntake(0.2)).withTimeout(0.1).finallyDo(this::stopIntake);
    }
}