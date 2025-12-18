// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;

/** Add your docs here. */
public class WiiBalanceBoard extends GenericHID {
    public WiiBalanceBoard(int port) {
        super(port);
    }
    public boolean isInUse() {
        return this.getRawAxis(3) > -0.95;
    }
    public double getYAxis() {
        return -MathUtil.applyDeadband(this.getRawAxis(1), 0.15);
    }
    public double getXAxis() {
        return MathUtil.applyDeadband(this.getRawAxis(0), 0.15);
    }
    

}
