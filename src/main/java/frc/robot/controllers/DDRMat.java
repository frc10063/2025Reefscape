// 7 year note when club is alive again: this ddrmat code has no comp value

package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DDRMat extends CommandJoystick {
    public DDRMat(int port) {
        super(port);
    }
    public Trigger getOrangeUpArrow() { 
        return button(3);
    }
    public Trigger getBlueUpArrow() { 
        return button(10);
    }
    public Trigger getOrangeDownArrow() { 
        return button(8);
    }
    public Trigger getBlueDownArrow() { 
        return button(14);
    }
    public Trigger getLeftArrow() {
        return button(13);
    }
    public Trigger getRightArrow() {
        return button(4);
    }
    public Trigger getOrangeCenterButton() { 
        return button(2);
    }
    public Trigger getBlueCenterButton() { 
        return button(15);
    }
    public Trigger getPlusButton() {
        return button(1);
    }
    public Trigger getMinusButton() {
        return button(16);
    }
    public int getBlueYValue() {
        int y = 0;
        if (getBlueUpArrow().getAsBoolean() && getBlueDownArrow().getAsBoolean()) {
            y = 0;
        } else if (getBlueUpArrow().getAsBoolean()) {
            y = 1;
        } else if (getBlueDownArrow().getAsBoolean()) {
            y = -1;
        } else {
            y = 0;
        }
        return y;
    }
    public int getBlueOrangeXValue() {
        int y = 0;
        if (getLeftArrow().getAsBoolean() && getRightArrow().getAsBoolean()) {
            y = 0;
        } else if (getLeftArrow().getAsBoolean()) {
            y = 1;
        } else if (getRightArrow().getAsBoolean()) {
            y = -1;
        } else {
            y = 0;
        }
        return y;
    }
    public double getMatYValue() {
        double y = 0;
        if ((getBlueUpArrow().getAsBoolean() && getBlueDownArrow().getAsBoolean()) || (getOrangeUpArrow().getAsBoolean() && getOrangeDownArrow().getAsBoolean())) {
            y = 0;
        } else if ((getBlueUpArrow().getAsBoolean() && getOrangeDownArrow().getAsBoolean())|| getOrangeUpArrow().getAsBoolean() && getBlueDownArrow().getAsBoolean()) {
            y = 0;
        } else if (getOrangeUpArrow().getAsBoolean() && getBlueUpArrow().getAsBoolean()) {
            y = 1;
        } else if (getOrangeDownArrow().getAsBoolean() && getBlueDownArrow().getAsBoolean()) {
            y = -1;
        } else if (getOrangeUpArrow().getAsBoolean() || getBlueUpArrow().getAsBoolean()) {
            y = 0.5;
        } else if (getOrangeDownArrow().getAsBoolean() || getBlueDownArrow().getAsBoolean()) {
            y = -0.5;
        }
        return y;
    }
    public double getMatXValue() {
        double x = 0;
        if (getLeftArrow().getAsBoolean() && getRightArrow().getAsBoolean()) {
            x = 0;
        } else if (getLeftArrow().getAsBoolean() && getBlueCenterButton().getAsBoolean()) {
            x = 1;
        } else if (getRightArrow().getAsBoolean() && getOrangeCenterButton().getAsBoolean()) {
            x = -1;
        } else if (getLeftArrow().getAsBoolean()) {
            x = 0.5;
        } else if (getRightArrow().getAsBoolean()) {
            x = -0.5;
        } else {
            x = 0;
        }
        return x;
    }
    public double getMatRotValue() {
        double rot = 0;
        if (getBlueUpArrow().getAsBoolean() && getOrangeDownArrow().getAsBoolean()) {
            rot = -1;
        } else if (getBlueDownArrow().getAsBoolean() && getOrangeUpArrow().getAsBoolean()) {
            rot = 1;
        } else {
            rot = 0;
        }
        return rot;
    }
}