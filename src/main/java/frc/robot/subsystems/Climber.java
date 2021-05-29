// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  
  private DoubleSolenoid pistons;
  private Compressor compressor;

  public Climber() {
    pistons = new DoubleSolenoid(Constants.ClimberConstants.FORWARD_CHANNEL, Constants.ClimberConstants.REVERSE_CHANNEL);
    compressor = new Compressor();
  }

  public void switchCompressor(boolean state) {
    if (state) {
      compressor.start();
    } else {
      compressor.stop();
    }
  }

  public void extendPiston(DoubleSolenoid.Value val) {
    pistons.set(val);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


}
