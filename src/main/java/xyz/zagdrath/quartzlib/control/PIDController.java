/*
 * @(#)PIDController.java
 * 
 * Copyright (c) 2022 Cody L. Wellman. All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found in the root
 * directory of this project.
 * 
 * Author: Cody L. Wellman (zagdrath@member.fsf.org)
 * 
 * Created: Mar 09, 2022
 * Updated: Mar 12, 2022
 */

package xyz.zagdrath.quartzlib.control;

public class PIDController {
    private double kP; // Proportional gain constant
    private double kI; // Integral gain constant
    private double kD; // Derivative gain constant

    private double kOutput; // Output of the controller

    private double kSetpoint; // Setpoint value

    private double kTolerance; // Tolerance for error

    private double kError; // Error value between setpoint and current value
    private double kLastError; // Error value from previous iteration

    private double kIntegralError; // Integral error
    private double kDerivativeError; // Derivative error

    private double kDeltaTime; // Time between iterations
    private double kLastDeltaTime; // Time between previous iterations

    private double kMinOutput = Double.NaN; // Minimum output value
    private double kMaxOutput = Double.NaN; // Maximum output value

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double calculatePID(double kCurrentTime, double kCurrentValue) {
        kDeltaTime = (kLastDeltaTime != Double.NaN) ? (kCurrentTime - kLastDeltaTime) : 0.0;

        kError = kSetpoint - kCurrentValue;
        
        kIntegralError += kError * kDeltaTime;

        kDerivativeError = (kDeltaTime != 0.0) ? (kError - kLastError) / kDeltaTime : 0.0;

        kLastError = kError;
        kLastDeltaTime = kCurrentTime;

        kOutput = checkOutputLimits((kP * kError) + (kI * kIntegralError) + (kD * kDerivativeError));

        return kOutput;
    }

    public void resetPID() {
        kError = 0.0;
        kLastError = 0.0;
    }

    public double getP() {
        return kP;
    }

    public double getI() {
        return kI;
    }

    public double getD() {
        return kD;
    }

    public void setP(double kP) {
        this.kP = kP;
    }

    public void setI(double kI) {
        this.kI = kI;
    }

    public void setD(double kD) {
        this.kD = kD;
    }

    public double getSetpoint() {
        return kSetpoint;
    }

    public void setSetpoint(double kSetpoint) {
        this.kSetpoint = kSetpoint;
    }

    public boolean atSetpoint() {
        if (kOutput == kSetpoint) {
            return true;
        } else {
            return false;
        }
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
