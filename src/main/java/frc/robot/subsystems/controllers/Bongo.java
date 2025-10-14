
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.controllers;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Bongo extends CommandJoystick {
    // When you see this in 7 years when the club is alive again,
    // this is the Gamecube DonkeyKong Bongo Controller
    // it has no competitive value but its cool
    public Bongo(int port) {
        super(port);
    }
    public Trigger getTopLeft() {
        return this.button(4);
    }
    public Trigger getBottomLeft() {
        return this.button(3);
    }
    public Trigger getTopRight() {
        return this.button(1);
    }
    public Trigger getBottomRight() {
        return this.button(2);
    }
    public Trigger getClap() {
        return this.axisGreaterThan(4, -0.09);
    }
    public Trigger getMiddleButton() {
        return this.button(10);
    }
    public Trigger getLeftFullBongo() {
        return getTopLeft().and(getBottomLeft());
    }
    public Trigger getRightFullBongo() {
        return getTopRight().and(getBottomRight());
    }

}
