// 7 year note when club is alive again: this ddrmat code has no comp value

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DDRMat extends CommandJoystick {
    public DDRMat(int port) {
        super(port);
    }
    public Trigger getOrangeUpArrow() { // might be 8
        return button(14);
    }
    public Trigger getBlueUpArrow() { // might be 14
        return button(8);
    }
    public Trigger getOrangeDownArrow() { // might be 3
        return button(10);
    }
    public Trigger getBlueDownArrow() { // might be 10
        return button(3);
    }
    public Trigger getLeftArrow() {
        return button(13);
    }
    public Trigger getRightArrow() {
        return button(4);
    }
    // public Trigger getOrangeCenterButton() { // unknown
    //     return button(0);
    // }
    // public Trigger getBlueCenterButton() { // unknown
    //     return button(0);
    // }
    public Trigger getPlusButton() {
        return button(1);
    }
    public Trigger getMinusButton() {
        return button(16);
    }
    
    // public Trigger getUpArrow() {
    //     return button(14||8);
    // }
    // public Trigger getDownArrow() {
    //     return button(10||3);
    // }
    public int getMatYValue() {
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
}