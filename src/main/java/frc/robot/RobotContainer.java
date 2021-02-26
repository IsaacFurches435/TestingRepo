// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveForward;
import frc.robot.commands.FollowTrajectory;
import frc.robot.commands.SetSafety;
import frc.robot.subsystems.DriveTrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public static DriveTrain train;
  public static XboxController controller;
  public static Joystick joy;

  public static SendableChooser<Integer> chooser;
  public static Button setSafeButton;
  public static FollowTrajectory trajectory;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    
    train = new DriveTrain();
    controller = new XboxController(OIConstants.XBOX_PORT);
    joy = new Joystick(OIConstants.JOY_PORT);
    
    
    
    chooser = new SendableChooser<>();

    

    setOptionsAndDrive();
    // Configure the button bindings
    configureButtonBindings();

    
  }

  private void setOptionsAndDrive() {
    chooser.setDefaultOption("Do nothing", 0);
    chooser.addOption("Follow Traject", 1);
    chooser.addOption("Drive Forward", 2);

    train.setDefaultCommand(
      new RunCommand(
        () -> train.drive(
          controller.getY(GenericHID.Hand.kLeft), 
          controller.getX(GenericHID.Hand.kLeft), 
          controller.getX(GenericHID.Hand.kRight), 
          false), train));
  }
  

  public SendableChooser<Integer> getChooser() {
    return chooser;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
    setSafeButton = new JoystickButton(controller, OIConstants.XBOX_SAFE_BUTTON_PORT);
    setSafeButton.toggleWhenPressed(new SetSafety());
    
  }

  
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    switch (chooser.getSelected()) {
      case 0:
        return null;
      case 1:
        return (new FollowTrajectory(train));
      case 2:
        return (new DriveForward(train));
    }
    return null;
  }
}
