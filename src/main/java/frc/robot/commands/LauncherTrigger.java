// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class LauncherTrigger extends Trigger {
  @Override
  public boolean get() {
    if (RobotContainer.controller.getTriggerAxis(Hand.kLeft) >= 1) {
      return true;
    }
    return false;
  }
}
