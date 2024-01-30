package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoTest extends SequentialCommandGroup {
    public AutoTest(SwerveSubsystem swerve) {
        TrajectoryConfig config = 
            new TrajectoryConfig(Constants.DrivebaseConstants.kMaxSpeedMetersPerSecond/2, 
            Constants.DrivebaseConstants.kTeleMaxDriveAccelerationMetersPerSecond)
            .setKinematics(Constants.DrivebaseConstants.kDriveKinematics);
        
        Trajectory exampleTrajectory = 
            // TrajectoryGenerator.generateTrajectory(
            //     new Pose2d(0, 0, new Rotation2d(0)), 
            //     List.of(new Translation2d(1, 1), new Translation2d(2, -1)), 
            //     new Pose2d(3, 0, new Rotation2d(0)), 
            //     config);
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)), 
                List.of(new Translation2d(1.5, 0)), 
                new Pose2d(3, 0, new Rotation2d(0)), 
                config);

        ProfiledPIDController thetaController = 
            new ProfiledPIDController(Constants.AutoConstants.kPAngleController, 
            0, 
            0, 
            Constants.AutoConstants.kAngleControllerConstraints);
            thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
        SwerveControllerCommand controllerCommand = 
            new SwerveControllerCommand(
            exampleTrajectory, 
            swerve::getPose, 
            Constants.DrivebaseConstants.kDriveKinematics, 
            new PIDController(Constants.AutoConstants.kPXController, 0, 0), 
            new PIDController(Constants.AutoConstants.kPYController, 0, 0), 
            thetaController, 
            swerve::setModules, 
            swerve);
        
        addCommands(new InstantCommand(() -> swerve.resetOdometry(exampleTrajectory.getInitialPose())), 
        controllerCommand);
    }
}
