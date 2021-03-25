// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherConstants;

public class Launcher extends SubsystemBase {
  
  
  public Launcher() {
    super();
    
    // launch = new PWMSparkMax(LauncherConstants.LAUNCH_PORT);
    
  }

  public double getAngle() {
    return -1;
  }

  @Override
  public void periodic() {
    
  }
}
