package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
    
    public WPI_TalonSRX lefTalonSRX;
    public WPI_TalonSRX lefTalonSRX2;
    public WPI_TalonSRX rigTalonSRX;
    public WPI_TalonSRX rigTalonSRX2;

    public MecanumDrive drive;

    public double xspeed;
    public double yspeed;
    private double turn; 

    
    public DriveTrain() {
        this.xspeed = 0;
        this.yspeed = 0;

        this.turn = 0;

        this.lefTalonSRX = new WPI_TalonSRX(Constants.LEFT_FRONT_MOTOR_PORT);
        this.rigTalonSRX = new WPI_TalonSRX(Constants.RIGHT_FRONT_MOTOR_PORT);
        this.lefTalonSRX2 = new WPI_TalonSRX(Constants.LEFT_FRONT_MOTOR_PORT);
        this.rigTalonSRX2 = new WPI_TalonSRX(Constants.RIGHT_BACK_MOTOR_PORT);

        this.drive = new MecanumDrive(this.lefTalonSRX, this.lefTalonSRX2, this.rigTalonSRX, this.rigTalonSRX2);
    }

    public void drive(ArrayList<Double> speed, double turn) {
        speed.add(this.xspeed);
        speed.add(this.yspeed);

        this.drive.driveCartesian(speed.get(0), speed.get(1), turn);
    }

    


    @Override
    public void periodic() {
        SmartDashboard.getNumberArray("Speed");
    }

}
