// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private WPI_TalonSRX elevator;
  public Elevator() {
    this.elevator = new WPI_TalonSRX(ElevatorConstants.ELEVATOR_PORT);
  }

  public void moveElevator(double state) {
    if (state > 0.5) {
      this.elevator.set(0.5);
    } else if (state < 0.5) {
      this.elevator.set(-0.5);
    } else {
      this.elevator.set(0);
    }
    
  }

  @Override
  public void periodic() {
    
  }
}
