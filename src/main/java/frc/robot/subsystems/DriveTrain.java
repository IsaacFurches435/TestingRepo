package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.IFollower;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;

import edu.wpi.first.wpilibj.kinematics.MecanumDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveTrain extends SubsystemBase {

    

    


    
    

    private double xSpeed;
    private double ySpeed;
    private double zSpeed;


    // Mecannum Wheel Motor Controllers
    private final SpeedController lefTalonSRX = new WPI_TalonSRX(DriveConstants.LEFT_FRONT_MOTOR_PORT);
    private final SpeedController lefTalonSRX2 = new WPI_TalonSRX(DriveConstants.LEFT_BACK_MOTOR_PORT);
    private final SpeedController rigTalonSRX = new WPI_TalonSRX(DriveConstants.RIGHT_FRONT_MOTOR_PORT);
    private final SpeedController rigTalonSRX2 = new WPI_TalonSRX(DriveConstants.RIGHT_BACK_MOTOR_PORT);

    

    // Drive Train Used
    private final MecanumDrive drivetrain = new MecanumDrive(lefTalonSRX, lefTalonSRX2, rigTalonSRX2, rigTalonSRX2);

    public Encoder lefEncoder1 = new Encoder(
        DriveConstants.FRONT_LEFT_ENCODER_PORTS[0], 
        DriveConstants.FRONT_LEFT_ENCODER_PORTS[1], 
        DriveConstants.FRONT_LEFT_ENCODER_REVERSE);
    
    public Encoder lefEncoder2 = new Encoder(
        DriveConstants.BACK_LEFT_ENCODER_PORTS[0],
        DriveConstants.BACK_LEFT_ENCODER_PORTS[1],
        DriveConstants.BACK_LEFT_ENCODER_REVERSE);

    public Encoder rigEncoder1 = new Encoder(
        DriveConstants.FRONT_RIGHT_ENCODER_PORTS[0],
        DriveConstants.FRONT_RIGHT_ENCODER_PORTS[1],
        DriveConstants.FRONT_RIGHT_ENCODER_REVERSE);

    public Encoder rigEncoder2 = new Encoder(
        DriveConstants.BACK_RIGHT_ENCODER_PORTS[0],
        DriveConstants.BACK_RIGHT_ENCODER_PORTS[1],
        DriveConstants.BACK_RIGHT_ENCODER_REVERSE);

    public final PIDController frontLeftController = new PIDController(1, 0, 0);
    public final PIDController backLeftController = new PIDController(1, 0, 0);
    public final PIDController frontRightController = new PIDController(1, 0, 0);
    public final PIDController backRightController = new PIDController(1, 0, 0);

    public final Gyro gyro = new AnalogGyro(0);
    private final MecanumDriveOdometry odometry = new MecanumDriveOdometry(DriveConstants.DRIVE_KINEMATICS, gyro.getRotation2d());

    ChassisSpeeds speeds = new ChassisSpeeds(1.0, 3.0, 1.5);
    public MecanumDriveWheelSpeeds wheelSpeeds = DriveConstants.DRIVE_KINEMATICS.toWheelSpeeds(speeds);

    public DriveTrain() {
        
        
        lefEncoder1.setDistancePerPulse(DriveConstants.ENCODER_DISTANCE_PER_PULSE);
        lefEncoder2.setDistancePerPulse(DriveConstants.ENCODER_DISTANCE_PER_PULSE);
        rigEncoder1.setDistancePerPulse(DriveConstants.ENCODER_DISTANCE_PER_PULSE);
        rigEncoder2.setDistancePerPulse(DriveConstants.ENCODER_DISTANCE_PER_PULSE);
            
        // Start With a clean state (Make sure everything is correctly instantiated.)
        // NOTE: I stole(borrowed) this from RI3D, may subject to change

        ((BaseMotorController) lefTalonSRX).configFactoryDefault();
        ((BaseMotorController) lefTalonSRX2).configFactoryDefault();

        ((TalonSRX) lefTalonSRX).configContinuousCurrentLimit(DriveConstants.CONTINUOUS_MAX_CURRENT);
        ((TalonSRX) lefTalonSRX2).configContinuousCurrentLimit(DriveConstants.CONTINUOUS_MAX_CURRENT);

        ((BaseMotorController) lefTalonSRX2).follow((IMotorController) lefTalonSRX);

        ((BaseMotorController) rigTalonSRX).configFactoryDefault();
        ((BaseMotorController) rigTalonSRX2).configFactoryDefault();

        ((TalonSRX) rigTalonSRX).configContinuousCurrentLimit(DriveConstants.CONTINUOUS_MAX_CURRENT);
        ((TalonSRX) rigTalonSRX2).configContinuousCurrentLimit(DriveConstants.CONTINUOUS_MAX_CURRENT);

        ((IFollower) rigTalonSRX2).follow((IMotorController) rigTalonSRX);

    }

    
    @Override
    public void periodic() {
        // Track the input values
        // NOTE: Not actual speed values (eg., RPM)
        SmartDashboard.putNumber("Xspeed", xSpeed);
        SmartDashboard.putNumber("Yspeed", ySpeed);
        SmartDashboard.putNumber("Zspeed", zSpeed);
        
        
    }

    public Pose2d getPos() {
        return odometry.getPoseMeters();
    }

    public void resetOdemetry(Pose2d pose) {
        odometry.resetPosition(pose, gyro.getRotation2d());
    }

    public void drive(double xspeed, double yspeed, double turn, boolean fieldRelative) {
        xSpeed = xspeed;
        ySpeed = -yspeed;
        zSpeed = turn;

        // determine wether you want to define the chassis speeds as being realtive from the field or by itself.
        DriveConstants.DRIVE_KINEMATICS.toWheelSpeeds(fieldRelative 
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xspeed, yspeed, turn, gyro.getRotation2d()) 
        : new ChassisSpeeds(xspeed, yspeed, turn));
        
    
    }


    public void resetEncoders() {
        lefEncoder1.reset();
        lefEncoder2.reset();
        rigEncoder1.reset();
        rigEncoder2.reset();
    }

    public Encoder getFrontLeftEncoder() {
        return lefEncoder1;
    }

    public Encoder getBackLeftEncoder() {
        return lefEncoder1;
    }

    public Encoder getFrontRightEncoder() {
        return lefEncoder1;
    }

    public Encoder getBackRightEncoder() {
        return lefEncoder1;
    }



    
    public MecanumDriveWheelSpeeds getCurrentWheelSpeeds() {
        return new MecanumDriveWheelSpeeds(
            lefEncoder1.getRate(),
            lefEncoder2.getRate(),
            rigEncoder1.getRate(),
            rigEncoder2.getRate());
    }
    

    public void setMaxOutput(double maxOutput) {
        drivetrain.setMaxOutput(maxOutput);
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return gyro.getRotation2d().getDegrees();
    }

    

    public double getTurnRate() {
        return gyro.getRate();
    }

    

    

    /**
     * Sets the desired speeds for each wheel
     * 
     */
    public void setSpeeds(MecanumDriveMotorVoltages volts) {


        lefTalonSRX.setVoltage(volts.frontLeftVoltage);
        lefTalonSRX2.setVoltage(volts.rearLeftVoltage);
        rigTalonSRX.setVoltage(volts.frontRightVoltage);
        rigTalonSRX2.setVoltage(volts.rearRightVoltage);

        
      }

    
    //   private void updateOdometry() {
    //     odometry.update(gyro.getRotation2d(), getCurrentWheelSpeeds());
      
    // }
    


}
