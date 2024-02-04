package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoTest extends SequentialCommandGroup {
    public AutoTest(SwerveSubsystem swerve) {
        TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.DrivebaseConstants.kDriveKinematics);
        
        Pose2d robotStartPose = new Pose2d(2.3, 5.65, new Rotation2d(0));
        Pose2d middleNotePose = new Pose2d(4.3, 5.65, new Rotation2d(0));
        Pose2d leftNotePose = new Pose2d(4.3, 7, new Rotation2d(0));

        Trajectory goToMiddleNote = 
         TrajectoryGenerator.generateTrajectory(
            robotStartPose,
            List.of(),
            middleNotePose,
            config);
        
        Trajectory goToSpeakerFromMiddleNote = 
            TrajectoryGenerator.generateTrajectory(
                middleNotePose, 
                List.of(new Translation2d(3, 5.6)), 
                robotStartPose,
                config);
        
        Trajectory goToLeftNote = 
            TrajectoryGenerator.generateTrajectory(
                robotStartPose, 
                List.of(new Translation2d(3.5, 7)),  // left and then fwd 
                leftNotePose, 
                config);

        Trajectory goToSpeakerFromLeftNote = 
            TrajectoryGenerator.generateTrajectory(
                leftNotePose, 
                List.of(), 
                robotStartPose, 
                config);

        // TODO SequentialCommandGroup that starts intake, goes to note, stops intake, returns to speaker, then shoots
        addCommands(
            // first and second notes
            new WaitCommand(1), // replace with shoot command
            new WaitCommand(1), // replace with start intake command
            new DriveTrajectory(swerve, goToMiddleNote), 
            new WaitCommand(1), // replace with stop intake command
            new DriveTrajectory(swerve, goToSpeakerFromMiddleNote), 
            new WaitCommand(1), // replace with shoot command
            // third (left) note
            new WaitCommand(1), // replace with start intake command
            new DriveTrajectory(swerve, goToLeftNote), 
            new WaitCommand(1), // replace with stop intake command
            new DriveTrajectory(swerve, goToSpeakerFromLeftNote), 
            new WaitCommand(1) // replace with shoot command
            );
    }
}
