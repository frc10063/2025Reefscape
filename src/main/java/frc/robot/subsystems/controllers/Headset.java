// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.controllers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Headset extends CommandXboxController {

    public Headset(int port) {
        super(port);
    }
    public Trigger pushButton() {
        return this.a();
    }
    public Trigger pullButton() {
        return this.b();
    }
}
