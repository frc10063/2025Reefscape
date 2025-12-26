// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class WiiBalanceBoard extends GenericHID {
    private int FL_AXIS_PORT = 2;
    private int FR_AXIS_PORT = 3;
    private int BL_AXIS_PORT = 4;
    private int BR_AXIS_PORT = 5;

    private double xOffset = 0.0;
    private double yOffset = 0.0;
    private double rotOffset = 0.0;

    private double DEADBAND = 0.15;
    public WiiBalanceBoard(int port) {
        super(port);
    }
    // Methods to get the raw axes from each corner
    public double getFL() {
        return this.getRawAxis(FL_AXIS_PORT);
    }
    public double getFR() {
        return this.getRawAxis(FR_AXIS_PORT);
    }
    public double getBL() {
        return this.getRawAxis(BL_AXIS_PORT);
    }
    public double getBR() {
        return this.getRawAxis(BR_AXIS_PORT);
    }
    /*
     * Method that adds one to each corner, sums up the weights, 
     * and creates ratios for each corner
     * Returns the normalized weights in an array
     */
    private double[] getNormalizedCorners() {
        double fl = getFL() + 1;
        double fr = getFR() + 1;
        double bl = getBL() + 1;
        double br = getBR() + 1;
        
        double total = fl + fr + bl + br;
        SmartDashboard.putNumber("weight", total);
        if (total < 0.05) {
            return new double[] {0.25, 0.25, 0.25, 0.25};
        }
        return new double[] {
            fl / total,
            fr / total, 
            bl / total,
            br / total
        };
    }

    // Returns X axis from weight ratios with right being +
    public double getXAxis() {
        double[] ratios = getNormalizedCorners();
        double fl = ratios[0];
        double fr = ratios[1];
        double bl = ratios[2];
        double br = ratios[3];
        SmartDashboard.putNumberArray("ratios", ratios);

        double x = (fr + br) - (fl + bl);
        x -= xOffset;
        x = MathUtil.applyDeadband(x, DEADBAND);
        SmartDashboard.putNumber("X board", x);
        return MathUtil.clamp(x, -1.0, 1.0);

    }

    // Returns Y axis from weight ratios with forward being +
    public double getYAxis() {
        double[] ratios = getNormalizedCorners();
        double fl = ratios[0];
        double fr = ratios[1];
        double bl = ratios[2];
        double br = ratios[3];

        double y = (fr + fl) - (br + bl);
        y -= yOffset;
        y = MathUtil.applyDeadband(y, DEADBAND);
        return MathUtil.clamp(y, -1.0, 1.0);
    }

    // Returns rotational axis where counterclockwise is +
    public double getRotAxis() {
        double[] ratios = getNormalizedCorners();
        double fl = ratios[0];
        double fr = ratios[1];
        double bl = ratios[2];
        double br = ratios[3];

        double rot = (fr + bl) - (br + fl);
        rot -= rotOffset;
        rot = MathUtil.applyDeadband(rot, DEADBAND);
        return MathUtil.clamp(rot, -1.0, 1.0);
    }

    public double applyResponseCurve(double input) {
        return Math.copySign(input * input, input);
    }


    public void calibrate() {
        double[] ratios = getNormalizedCorners();
        double fl = ratios[0];
        double fr = ratios[1];
        double bl = ratios[2];
        double br = ratios[3];

        xOffset = (fr + br) - (fl + bl);
        yOffset = (fl + fr) - (bl + br);
        rotOffset = (fr + bl) - (fl + br);
    }


    public boolean isInUse() {
        double fl = getFL();
        double fr = getFR();
        double bl = getBL();
        double br = getBR();

        double total = fl + fr + bl + br;
        return (total > 0.05);
    }

    public boolean isCalibrationPressed() {
        return getRawButtonPressed(1);
    }
    public Trigger calibrationButton() {
        return new Trigger(this::isCalibrationPressed);
    }
}
