package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.IFollower;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class DriveTrain extends SubsystemBase {

    private SpeedController lefTalonSRX;
    private SpeedController lefTalonSRX2;
    private SpeedController rigTalonSRX;
    private SpeedController rigTalonSRX2;

    private Encoder lefEncoder1;
    private Encoder rigEncoder1;
    private Encoder lefEncoder2;
    private Encoder rigEncoder2;

    private PIDController lefController; // Possibly Use
    private PIDController rigController; // Possibly Use

    private SimpleMotorFeedforward lefFF; // Possibly Use
    private SimpleMotorFeedforward rigFF; // Possibly Use

    private MecanumDriveKinematics kinematics;
    private MecanumDriveOdometry odometry;

    private MecanumDrive drive;

    private double xSpeed;
    private double ySpeed;
    private double zSpeed;

    public DriveTrain() {
        super();
        // Mecannum Wheel Motor Controllers
        lefTalonSRX = new WPI_TalonSRX(Constants.LEFT_FRONT_MOTOR_PORT);
        lefTalonSRX2 = new WPI_TalonSRX(Constants.LEFT_BACK_MOTOR_PORT);
        rigTalonSRX = new WPI_TalonSRX(Constants.RIGHT_FRONT_MOTOR_PORT);
        rigTalonSRX2 = new WPI_TalonSRX(Constants.RIGHT_BACK_MOTOR_PORT);

        Translation2d tran = new Translation2d();

        // Drive Train Used
        drive = new MecanumDrive(lefTalonSRX, lefTalonSRX2, rigTalonSRX2, rigTalonSRX2);

        // Start With a clean state (Make sure everything is correctly instantiated.)
        // NOTE: I stole(borrowed) this from RI3D, may subject to change

        ((BaseMotorController) lefTalonSRX).configFactoryDefault();
        ((BaseMotorController) lefTalonSRX2).configFactoryDefault();

        ((TalonSRX) lefTalonSRX).configContinuousCurrentLimit(Constants.CONTINUOUS_MAX_CURRENT);
        ((TalonSRX) lefTalonSRX2).configContinuousCurrentLimit(Constants.CONTINUOUS_MAX_CURRENT);

        ((BaseMotorController) lefTalonSRX2).follow((IMotorController) lefTalonSRX);

        ((BaseMotorController) rigTalonSRX).configFactoryDefault();
        ((BaseMotorController) rigTalonSRX2).configFactoryDefault();

        ((TalonSRX) rigTalonSRX).configContinuousCurrentLimit(Constants.CONTINUOUS_MAX_CURRENT);
        ((TalonSRX) rigTalonSRX2).configContinuousCurrentLimit(Constants.CONTINUOUS_MAX_CURRENT);

        ((IFollower) rigTalonSRX2).follow((IMotorController) rigTalonSRX);

    }

    public void drive(double xspeed, double yspeed, double turn) {
        xSpeed = xspeed;
        ySpeed = -yspeed;
        zSpeed = turn;

        drive.driveCartesian(ySpeed, xSpeed, zSpeed, 0.0);
    }

    

    @Override
    public void periodic() {
        // Track the input values
        // NOTE: Not actual speed values (eg., RPM)
        SmartDashboard.putNumber("Xspeed", xSpeed);
        SmartDashboard.putNumber("Yspeed", ySpeed);
        SmartDashboard.putNumber("Zspeed", zSpeed);
        
    }

}
