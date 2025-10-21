// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** Add your docs here. */
public class DualPIDController {
    public final double maxVelocity;
    public final double maxAcceleration;
    public final double maxDeceleration;
    public final TrapezoidProfile accelerationConstraints;
    public final TrapezoidProfile decelerationConstraints;
    public DualPIDController(double maxVelocity, double maxAcceleration, double maxDeceleration) {
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.maxDeceleration = maxDeceleration;
        accelerationConstraints = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
        decelerationConstraints = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVelocity, maxDeceleration));
        
    }
}
