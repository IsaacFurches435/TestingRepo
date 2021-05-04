// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.RobotContainer;



public class Launcher extends SubsystemBase {

    private CANSparkMax launching;
    private Spark intake;
    private VictorSPX pivot;

    private DigitalInput topdigital;
    private DigitalInput bottomdigital;

    /**
     * Creates a Launcher object to track and manage the movement of the launcher.
     */
    public Launcher() {
        launching = new CANSparkMax(Constants.LauncherConstants.LAUNCH_PORT, MotorType.kBrushless);
        intake = new Spark(Constants.LauncherConstants.INTAKE_PORT);
        pivot = new VictorSPX(Constants.LauncherConstants.PIVOT_PORT);
        topdigital = new DigitalInput(Constants.LauncherConstants.DIGITAL_INPUT);
        bottomdigital = new DigitalInput(Constants.LauncherConstants.DIGITAL_INPUT_2);
    }
    /**
     * returns the state of the top digital input as well as stops the pivot motor from moving
     * @return the state of the top-digital input
     */
    private boolean getTopSwitchState() {
        if (topdigital.get() == true) {
            stopMotor();
            return topdigital.get(); 
        }
        return topdigital.get();
    }
    /**
     * returns the state of the bottom digital input as well as stops the pivot motor from moving
     * @return the state of bottom-digital input
     */
    private boolean getBottomSwitchState() {
        if (bottomdigital.get() == true) {
            stopMotor();
            return bottomdigital.get();
        }
        return bottomdigital.get();
    }
    
    /**
     * rotates the launcher in either the clockwise or counter-clockwise rotation.  
     */
    public void rotatePiviot(int state) {
        // state is either 0 OR 180
        switch(state) {
            case 0:
                pivot.set(ControlMode.PercentOutput, 80);
            case 180:
                pivot.set(ControlMode.PercentOutput, -80);
        }
        stopMotor();
    }
    
    /**
     * launches ball whenever the trigger is pressed
     * @param triggerValue the value of the trigger 0.0-1.0
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
    public void stopMotor() {
        pivot.set(ControlMode.PercentOutput, 0);
    }

    

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Top State", getTopSwitchState());
        SmartDashboard.putBoolean("Bottom State", getBottomSwitchState());
    }

    
}

