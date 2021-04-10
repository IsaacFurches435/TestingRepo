// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.can.VictorSPX;

// import edu.wpi.first.wpilibj.AnalogInput;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj.PWMSparkMax;
// import edu.wpi.first.wpilibj.Spark;
// import edu.wpi.first.wpilibj.SpeedController;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.RobotContainer;
// import frc.robot.Constants.LauncherConstants;
// import frc.robot.Constants.OtherConstants;

// public class Launcher extends SubsystemBase {
  
//   public PWMSparkMax launch;
//   public VictorSPX pivot;
//   public Spark intake;
//   public DigitalInput input;
//   public Encoder encoder;
//   public DigitalInput input2;
//   public AnalogInput supersensor;
  
//   public Launcher() {
//     super();
    

//     launch = new PWMSparkMax(LauncherConstants.LAUNCH_PORT);
//     intake = new Spark(LauncherConstants.INTAKE_PORT);
//     pivot = new VictorSPX(LauncherConstants.PIVOT_PORT);
//     input = new DigitalInput(0);
//     encoder = new Encoder(10, 11);
//     input2 = new DigitalInput(8);
//     supersensor = new AnalogInput(1);
     

//   }

//   public void setAngle(double percentoutput) {
//     if (RobotContainer.controller.getAButton()) {
//       pivot.set(ControlMode.PercentOutput, percentoutput);
//     } else if (RobotContainer.controller.getBButton()) {
//       pivot.set(ControlMode.PercentOutput, -percentoutput);
//     }
//   }

//   public boolean getDigitalState(DigitalInput input) {
//     return input.get();
//   }


//   private double getAngle(boolean isEncoder) {
//     double angle = (encoder.getRaw() / 2408d) * 360d;
//     if (isEncoder) {
//       SmartDashboard.putBoolean("AngleEncoder", isEncoder);
//       return angle;
//     }
//     return 0;
//   }

//   public void stopMotor() {
//     if (input.get() == true || input2.get() == true) {
//       pivot.set(ControlMode.PercentOutput, 0);
//     }
//   }

  

//   @Override
//   public void periodic() {
//     stopMotor();
//     SmartDashboard.putNumber("Get angle", getAngle(false));
//   }
// }
