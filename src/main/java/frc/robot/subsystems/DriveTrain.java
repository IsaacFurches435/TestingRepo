package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class DriveTrain extends SubsystemBase {
    
    private WPI_TalonSRX lefTalonSRX;
    private WPI_TalonSRX lefTalonSRX2;
    private WPI_TalonSRX rigTalonSRX;
    private WPI_TalonSRX rigTalonSRX2;

    private Encoder lefEncoder;
    private Encoder rigEncoder;

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
       
       
       
       // Drive Train Used
       drive = new MecanumDrive(lefTalonSRX, lefTalonSRX2, rigTalonSRX2, rigTalonSRX2);

       // Start With a clean state (Make sure everything is correctly instantiated.)
       // NOTE: I stole(borrowed) this from RI3D, may subject to change

       lefTalonSRX.configFactoryDefault();
       lefTalonSRX2.configFactoryDefault();

       lefTalonSRX.configContinuousCurrentLimit(Constants.CONTINUOUS_MAX_CURRENT);
       lefTalonSRX2.configContinuousCurrentLimit(Constants.CONTINUOUS_MAX_CURRENT);

       lefTalonSRX2.follow(lefTalonSRX);

       rigTalonSRX.configFactoryDefault();
       rigTalonSRX2.configFactoryDefault();

       rigTalonSRX.configContinuousCurrentLimit(Constants.CONTINUOUS_MAX_CURRENT);
       rigTalonSRX2.configContinuousCurrentLimit(Constants.CONTINUOUS_MAX_CURRENT);

       rigTalonSRX2.follow(rigTalonSRX);

    }

    public void drive(double xspeed, double yspeed, double turn) {
        xSpeed = xspeed;
        ySpeed = -yspeed;
        zSpeed = turn; 
        drive.driveCartesian(ySpeed, xSpeed, zSpeed);
    }

    

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Xspeed", xSpeed);
        SmartDashboard.putNumber("Yspeed", ySpeed);
        
    }

}
