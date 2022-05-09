/*
 * @(#)PIDController.java
 * 
 * Copyright (c) 2022 Cody L. Wellman. All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found in the root
 * directory of this project.
 * 
 * Author: Cody L. Wellman (zagdrath@member.fsf.org)
 * 
 * Created: Mar 09, 2022 Updated: Mar 09, 2022
 */

package xyz.zagdrath.quartzlib.control;

public class PIDController {
    private double kP; // Proportional gain constant
    private double kI; // Integral gain constant
    private double kD; // Derivative gain constant

    private double kSetpoint; // Setpoint value

    private double kMinOutput = Double.NaN; // Minimum output value
    private double kMaxOutput = Double.NaN; // Maximum output value

    private double kError; // Error value between setpoint and current value
    private double kLastError; // Error value from previous iteration

    private double kDeltaTime; // Time between iterations
    private double kLastTime; // Time between previous iterations

    private double kIntegralError; // Integral error value
    private double kDerivativeError; // Derivative error value

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double calculatePID(double kCurrentTime, double kCurrentValue) {
        kError = kSetpoint - kCurrentValue;
        kDeltaTime = (kLastTime != Double.NaN) ? (double) (kCurrentTime - kLastTime) : 0.0;

        kDerivativeError = (kDeltaTime != 0.0) ? (kError - kLastError) / kDeltaTime : 0.0;

        kIntegralError += kError * kDeltaTime;

        kLastError = kError;
        kLastTime = kCurrentTime;

        return checkOutputLimits((kP * kError) + (kI * kIntegralError) + (kD * kDerivativeError));
    }

    public double getkP() {
        return kP;
    }

    public void setkP(double kP) {
        this.kP = kP;
    }

    public double getkI() {
        return kI;
    }

    public void setkI(double kI) {
        this.kI = kI;
    }

    public double getkD() {
        return kD;
    }

    public void setkD(double kD) {
        this.kD = kD;
    }

    public double getkSetpoint() {
        return kSetpoint;
    }

    public void setkSetpoint(double kSetpoint) {
        this.kSetpoint = kSetpoint;
    }

    private double checkOutputLimits(double kOutput) {
        if (!Double.isNaN(kMinOutput) && kOutput < kMinOutput) {
            return kMinOutput;
        } else if (!Double.isNaN(kMaxOutput) && kOutput > kMaxOutput) {
            return kMaxOutput;
        } else {
            return kOutput;
        }
    }

    public void setOutputLimits(double kMinOutput, double kMaxOutput) {
        if (kMinOutput > kMaxOutput) {
            this.kMinOutput = kMinOutput;
            this.kMaxOutput = kMaxOutput;
        } else {
            this.kMinOutput = kMaxOutput;
            this.kMaxOutput = kMinOutput;
        }
    }
}
