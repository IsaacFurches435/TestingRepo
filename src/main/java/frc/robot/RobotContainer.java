// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;

// import frc.robot.commands.FollowTrajectory;
import frc.robot.commands.SetSafety;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Launcher;
// import frc.robot.subsystems.Launcher;
import frc.robot.util.JoystickAxis;

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
  public static Button switchMode;
  
  public static SendableChooser<Integer> mode_chooser;

  public static JoystickAxis xAxis;
  public static JoystickAxis yAxis;
  public static JoystickAxis zAxis;

  public static JoystickButton intakeButton;
  public static POVButton pivotButton;
  public static Trigger launchTrigger;

  public static DoubleSupplier zSupplier;
  public static DoubleSupplier xSupplier;
  public static DoubleSupplier ySupplier;

  public static Launcher launch;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    
    train = new DriveTrain();
    controller = new XboxController(OIConstants.XBOX_PORT);
    joy = new Joystick(OIConstants.JOY_PORT);
    launch = new Launcher();

    mode_chooser = new SendableChooser<>();
    
    chooser = new SendableChooser<>();

    yAxis = new JoystickAxis(joy, 0, 5.6, 10, 0, 0.1);
    xAxis = new JoystickAxis(joy, 1, 3.2, 10, 0, 0.1);
    zAxis = new JoystickAxis(joy, 2, 2.0, 10, 0, 0.1);

    ySupplier = () -> yAxis.get();
    xSupplier = () -> xAxis.get();
    zSupplier = () -> zAxis.get();
    

    setOptionsAndDrive();
    // Configure the button bindings
    configureButtonBindings();

    
  } 

  private void setOptionsAndDrive() {
    chooser.setDefaultOption("Do nothing", 0);
    chooser.addOption("Follow Traject", 1);
    chooser.addOption("Drive Forward", 2);

    mode_chooser.setDefaultOption("Normal Mode", 0);
    mode_chooser.addOption("Normal Mode", 0);
    mode_chooser.addOption("Encoder Mode", 1);

    // train.setDefaultCommand(
    //   new RunCommand(
    //     () -> train.drive(
    //       controller.getY(GenericHID.Hand.kLeft), 
    //       controller.getX(GenericHID.Hand.kLeft), 
    //       controller.getX(GenericHID.Hand.kRight), 
    //       false), train));

    
  }
  

  public SendableChooser<Integer> getChooser() {
    return chooser;
  }

  public SendableChooser<Integer> getModeChooser() {
    return mode_chooser;
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

    intakeButton = new JoystickButton(controller, XboxController.Button.kA.value);
    intakeButton.toggleWhenPressed(new RunCommand(() -> launch.intakeBall(intakeButton, 1.0)));

    pivotButton = new POVButton(controller, controller.getPOV());
    pivotButton.toggleWhenPressed(new RunCommand(() -> launch.rotatePiviot(controller.getPOV())));

    launchTrigger = new Trigger();
    launchTrigger.toggleWhenActive(new RunCommand(() -> launch.launchBall(1.0)));
  }

  public static int getMode() {
    switch(mode_chooser.getSelected()) {
      case 0:
        return 0;
      case 1:
        return 1;
    }
    return -1;
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
        // return (new FollowTrajectory(train));
      case 2:
        // return (new DriveForward(train));
    }
    return null;
  }
}

