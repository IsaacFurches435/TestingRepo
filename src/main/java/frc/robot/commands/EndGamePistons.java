// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class EndGamePistons extends CommandBase {
  /** Creates a new EndGamePistons. */
  public EndGamePistons() {
    addRequirements(new Climber());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.climber.extendPiston(Value.kOff);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.controller.getPOV() == -1){
       
    } else if(RobotContainer.controller.getPOV() <= 45 || RobotContainer.controller.getPOV() >= 345){
      RobotContainer.climber.extendPiston(Value.kForward);
    } else if (RobotContainer.controller.getPOV() >= 135 && RobotContainer.controller.getPOV() <= 225){
      RobotContainer.climber.extendPiston(Value.kReverse);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
