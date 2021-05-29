// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/** Add your docs here. */
public class DistanceSensor {
    private static final AnalogInput mb1013 = new AnalogInput(Constants.OtherConstants.SENSOR_PORT);
    
    public static double getValue() {
        return mb1013.getVoltage();
    }

    public static double getDistance() {
        return getValue() * Constants.OtherConstants.VALUE_TO_INCHES;
    }

    public static void updateDashboard() {
        SmartDashboard.putNumber("Distance from backboard (inch): ", getDistance());
    }
}
