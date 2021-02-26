package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrain;

public class FollowTrajectory extends MecanumControllerCommand {
    

    public FollowTrajectory(DriveTrain driveTrain) {
        
        super(  
                getTrajectory(), 
                driveTrain::getPos, 
                DriveConstants.FEED_FORWARD, 
                DriveConstants.DRIVE_KINEMATICS,
                new PIDController(AutoConstants.P_X_CONTROLLER, 0, 0),
                new PIDController(AutoConstants.P_Y_CONTROLLER, 0, 0),
                new ProfiledPIDController(AutoConstants.P_THETA_CONTROLLER, 0, 0, AutoConstants.CONSTRAINTS),
                AutoConstants.MAX_SPEED,
                driveTrain.frontLeftController,
                driveTrain.backLeftController,
                driveTrain.frontRightController,
                driveTrain.backRightController,
                driveTrain::getCurrentWheelSpeeds,
                driveTrain::setSpeeds,
                driveTrain);
                
        
        driveTrain.resetOdemetry(getTrajectory().getInitialPose());
        driveTrain.resetEncoders();
        

    }

    /**
     * A method used to return a trajectory from a JSON file
     * @return Trajectory
     */
    private static Trajectory getTrajectory() {
        String trajectoryJSON1 = "paths/Red_Path1.wpilib.json";
        // String trajectoryJSON2 = "paths/Red_path2.wpilib.json";
        // String trajectoryJSON3 = "paths/Red_path3.wpilib.json";
        Trajectory trajectory = new Trajectory();
        try {
            Path trajectoryPath1 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON1);
            
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath1);
        } catch(IOException e) {
            DriverStation.reportError("Unable to open trajectory", e.getStackTrace());
        }

        return trajectory;
    }
}   