// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants;
import frc.robot.util.Map;




public class Launcher extends SubsystemBase {

    private CANSparkMax launching;
    private WPI_TalonSRX intake;
    private VictorSPX pivot;

    private DigitalInput topdigital;
    private DigitalInput bottomdigital;

    private Encoder angleRotation;
    
    private Map controllerToPercentageMap;

    

    /**
     * Creates a Launcher object to track and manage the movement of the launcher.
     */
    public Launcher() { 
        launching = new CANSparkMax(Constants.LauncherConstants.LAUNCH_PORT, MotorType.kBrushless);
        intake = new WPI_TalonSRX(Constants.LauncherConstants.INTAKE_PORT);
        pivot = new VictorSPX(Constants.LauncherConstants.PIVOT_PORT);
        topdigital = new DigitalInput(Constants.LauncherConstants.DIGITAL_INPUT);
        bottomdigital = new DigitalInput(Constants.LauncherConstants.DIGITAL_INPUT_2);
        angleRotation = new Encoder(Constants.LauncherConstants.PIVOT_ENCODER_PORTS[0], Constants.LauncherConstants.PIVOT_ENCODER_PORTS[1]);
        
        controllerToPercentageMap = new Map(-1, 1, -100, 100);

        angleRotation.setDistancePerPulse(Constants.LauncherConstants.DISTANCE_PER_PULSE);
        angleRotation.setMaxPeriod(1.5);
        
    }
    /**
     * returns the state of the top digital input as well as stops the pivot motor from moving
     * @return the state of the top-digital input
     */
    private boolean getTopSwitchState() {
        if (topdigital.get()) {
            // stopMotor();
            return true;
        }
        return false;
    }
    /**
     * returns the state of the bottom digital input as well as stops the pivot motor from moving
     * @return the state of bottom-digital input
     */
    private boolean getBottomSwitchState() {
        if (bottomdigital.get()) {
            // stopMotor();
            return true;
        }
        return false;
    }
    
    /**
     * rotates the launcher in either the clockwise or counter-clockwise rotation.
     * @param state is either 0 or 180
     */
    public void rotatePiviot(double state) {
        // switch(state) {
        //     case 0:
        //         if (getBottomSwitchState()) {
        //             System.out.println("          1");
        //             pivot.set(ControlMode.PercentOutput, 0);
        //         } else {
        //             System.out.println("          2");
        //             pivot.set(ControlMode.PercentOutput, 80);
        //         }
        //         break;
                
        //     case 180:
        //         if (getTopSwitchState()) {
        //             System.out.println("          3");
        //             pivot.set(ControlMode.PercentOutput, 0);
        //         } else {
        //             System.out.println("          4");
        //             pivot.set(ControlMode.PercentOutput, -80);
        //         }
        //         break;
        //     default:
        //         System.out.println("          5");
        //         pivot.set(ControlMode.PercentOutput, 0);
        // }

        if (state < 0) {
            if (getBottomSwitchState()) {
                System.out.println("          1");
                pivot.set(ControlMode.PercentOutput, 0);
            } else {
                System.out.println("          2");
                pivot.set(ControlMode.PercentOutput, controllerToPercentageMap.getNewValue(-state)); // reverses state beacuse controller joystick is opp of what is expected
            }
        } else if (state > 0) {
            if (getTopSwitchState()) {
                System.out.println("          3");
                pivot.set(ControlMode.PercentOutput, 0);
            } else {
                System.out.println("          4");
                pivot.set(ControlMode.PercentOutput, controllerToPercentageMap.getNewValue(-state));
            }
        } else {
            System.out.println("          5");
            pivot.set(ControlMode.PercentOutput, 0);
        }
        
    }
    
    /**
     * launches ball whenever the trigger is pressed
     * @param speed the speed in which the launcher is moving
     */
    public void launchBall (double speed) {
        launching.set(speed);
    }

    /**
     * intakes the ball whenever
     * @param buttonTrigger the button used to trigger this method (A)
     * @param speed the speed in which the intake will move the ball to launcher
     */
    public void intakeBall(JoystickButton buttonTrigger, double speed) {
        if (buttonTrigger.get() == true) {
            intake.set(speed);
        }
    }

    /**
     * stops the pivot motor from moving
     */
    // public void stopMotor() {
    //     pivot.set(ControlMode.PercentOutput, 0);
    // }

    private double getAngle() {
        
        double angle = angleRotation.getDistance();
        
        if (getBottomSwitchState() == true) {
            angleRotation.reset();
            
            angle = angleRotation.getDistance() + 20;
        }

        if (getTopSwitchState() == true) {
            angleRotation.reset();
            angle = angleRotation.getDistance() + 40;
        } 
        
        if (getBottomSwitchState() == false && getTopSwitchState() == false) {
            if (pivot.getMotorOutputVoltage() < 0) {
                angle += angleRotation.getDistance();
            } else {
                angle -= angleRotation.getDistance();
            }
        }

        if (angleRotation.getStopped() == true) {
            // stopMotor();
        }

        return angle % 360;
    }
    

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Top State", getTopSwitchState());
        SmartDashboard.putBoolean("Bottom State", getBottomSwitchState());
        SmartDashboard.putNumber("Pivot Angle", getAngle());

    }

    
}

