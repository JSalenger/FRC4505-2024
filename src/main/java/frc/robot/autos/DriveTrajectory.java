package frc.robot.autos;

import java.io.IOException;
import java.nio.file.FileSystem;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveTrajectory extends SequentialCommandGroup {
    public DriveTrajectory(SwerveSubsystem swerve, Trajectory trajectory) {
        TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.DrivebaseConstants.kDriveKinematics);

    // String trajectoryJSON = "paths/auto_test.wpilib.json";
    // An example trajectory to follow.  All units in meters.
    // Trajectory exampleTrajectory =

        // new Trajectory();
        // try {
        //     Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        //     exampleTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        // } catch (IOException ex) {
        //     DriverStation.reportError("unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        // }
        // TrajectoryGenerator.generateTrajectory(
        //     // Start at the origin facing the +X direction
        //     new Pose2d(0, 0, new Rotation2d(0)),
        //     // Pass through these two interior waypoints, making an 's' curve path
        //     List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        //     // End 3 meters straight ahead of where we started, facing forward
        //     new Pose2d(3, 0, new Rotation2d(0)),
        //     config);

        // TrajectoryGenerator.generateTrajectory(
        //     new Pose2d(0, 0, new Rotation2d(0)),
        //     List.of(),
        //     new Pose2d(1, 0, new Rotation2d(0)),
        //     config);

        // TrajectoryGenerator.generateTrajectory(
        //     new Pose2d(0, 0, new Rotation2d(0)),
        //     List.of(
        //         new Translation2d(1, 0), 
        //         new Translation2d(1, 1), 
        //         new Translation2d(0, 1)),
        //     new Pose2d(0, 0, new Rotation2d(0)),
        //     config);  // correctly returns to original position but most other parts are wrong
        
        // // retrieve middle note
        // TrajectoryGenerator.generateTrajectory(
        //     new Pose2d(2.3, 5.65, new Rotation2d(0)),
        //     List.of(
        //         new Translation2d(4, 5), 
        //         new Translation2d(5, 5.65)),
        //     new Pose2d(2.3, 5.65, new Rotation2d(0)),
        //     config);
    var thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.kPAngleController,
            0,
            0,
            Constants.AutoConstants.kAngleControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            trajectory,
            swerve::getPose,
            Constants.DrivebaseConstants.kDriveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0.01, 0.001),
            new PIDController(Constants.AutoConstants.kPYController, 0.01, 0.001),
            thetaController,
            swerve::setModules,
            swerve);
        addRequirements(swerve);
        
            addCommands(
        new InstantCommand(() -> swerve.resetOdometry(trajectory.getInitialPose())),
        swerveControllerCommand);
  }
}
