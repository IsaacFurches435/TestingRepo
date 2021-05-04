// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    

    
    
    
    
    
    public static final class DriveConstants {

        /**
         *  ------------- VARIABLE DRIVE CONSTANTS ---------------
         */
        public static final int LEFT_FRONT_MOTOR_PORT = 1;
        public static final int LEFT_BACK_MOTOR_PORT = 2;
        public static final int RIGHT_FRONT_MOTOR_PORT = 3;
        public static final int RIGHT_BACK_MOTOR_PORT = 4;

        

        public static final int CONTINUOUS_MAX_CURRENT = 40;

        public static final int[] FRONT_LEFT_ENCODER_PORTS = new int[] {0, 1};
        public static final int[] BACK_LEFT_ENCODER_PORTS = new int[] {2, 3};
        public static final int[] FRONT_RIGHT_ENCODER_PORTS = new int[] {4, 5};
        public static final int[] BACK_RIGHT_ENCODER_PORTS = new int[] {6, 7};

        
        public static final boolean FRONT_LEFT_ENCODER_REVERSE = false;
        public static final boolean BACK_LEFT_ENCODER_REVERSE = true;
        public static final boolean FRONT_RIGHT_ENCODER_REVERSE = true;
        public static final boolean BACK_RIGHT_ENCODER_REVERSE = false;

        public static final double TRACK_WIDTH = 0.55;
        public static final double WHEEL_BASE = 0.75;

        public static final MecanumDriveKinematics DRIVE_KINEMATICS = new MecanumDriveKinematics(
            new Translation2d(WHEEL_BASE/2, TRACK_WIDTH/2),
            new Translation2d(WHEEL_BASE/2, -TRACK_WIDTH/2),
            new Translation2d(-WHEEL_BASE/2, TRACK_WIDTH/2),
            new Translation2d(-WHEEL_BASE, -TRACK_WIDTH));

        public static final int ENCODER_COUNTS_PER_REVOLUTION = 2048;
        public static final double WHEEL_DIAMETER_METERS = 0.17;

        public static final double ENCODER_DISTANCE_PER_PULSE = 
            (WHEEL_DIAMETER_METERS * Math.PI) / (double) ENCODER_COUNTS_PER_REVOLUTION;

        public static final SimpleMotorFeedforward FEED_FORWARD =
            new SimpleMotorFeedforward(1, 0.8, 0.15);
        
        public static final double P_FRONT_LEFT_VEL = 0.5;
        public static final double P_BACK_LEFT_VEL = 0.5;
        public static final double P_FRONT_RIGHT_VEL = 0.5;
        public static final double P_BACK_RIGHT_VEL = 0.5;

    }

    public static final class OIConstants {
        public static final int JOY_PORT = 0;
        public static final int XBOX_PORT = 1;
        public static final int XBOX_SAFE_BUTTON_PORT = 1;
        public static final int JOY_SWITCH_MODE_BUTTON = 3;
        
    }

    public static final class AutoConstants {

        public static final double MAX_SPEED = 3.0;
        public static final double MAX_ACCEL = 3.0;
        public static final double MAX_ANGULAR_RADIANS = Math.PI;
        public static final double MAX_ANGULAR_RADIANS_SQUARED = Math.PI;
        
        public static final double P_X_CONTROLLER = 0.5;
        public static final double P_Y_CONTROLLER = 0.5;
        public static final double P_THETA_CONTROLLER = 0.5;

        public static final TrapezoidProfile.Constraints CONSTRAINTS =
            new TrapezoidProfile.Constraints(
                MAX_ANGULAR_RADIANS, MAX_ANGULAR_RADIANS_SQUARED);
    }

    public static final class LauncherConstants {
        public static final int LAUNCH_PORT = 8;
        public static final int PIVOT_PORT = 6;
        public static final int INTAKE_PORT = 0;
        public static final int DIGITAL_INPUT = 8;
        public static final int DIGITAL_INPUT_2 = 9;
        public static final int[] PIVOT_ENCODER_PORTS = new int[] {10, 11};

        public static final int PIVOT_PULSES_PER_REVOLUTION = 2048;
        public static final double DISTANCE_PER_PULSE = 360f / PIVOT_PULSES_PER_REVOLUTION;

        public static final int CONTINUOUS_MAX_CURRENT = 20;

    }

    public static final class ElevatorConstants {
        public static final int ELEVATOR_PORT = 7;
    }

    public static final class OtherConstants {
        public static final int SENSOR_PORT = 0;
    }
}
