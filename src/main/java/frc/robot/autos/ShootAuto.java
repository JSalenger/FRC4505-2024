package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.ReverseNoteCommand;
import frc.robot.commands.SetShooterCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class ShootAuto extends SequentialCommandGroup {
    public ShootAuto(SwerveSubsystem swerve, ShooterSubsystem shooter, IntakeSubsystem intake) {
        TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.DrivebaseConstants.kDriveKinematics);
        
        Pose2d robotStartPose = new Pose2d(2.3, 5.65, new Rotation2d(0));
        Pose2d middleNotePose = new Pose2d(4.3+2, 5.65, new Rotation2d(0));
        Pose2d leftNotePose = new Pose2d(4.3, 7, new Rotation2d(0));
        Pose2d speakerPoseWithMargin = new Pose2d(2.3, 5.65-0.1, new Rotation2d(0));  // add margin so robot doesn't run into speaker
        //x 2.5
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
                speakerPoseWithMargin,  // robotStartPose
                config);
        
        Trajectory goToLeftNote = 
            TrajectoryGenerator.generateTrajectory(
                robotStartPose, 
                List.of(new Translation2d(3.2, 7)),  // left and then fwd 
                leftNotePose, 
                config);

        Trajectory goToSpeakerFromLeftNote = 
            TrajectoryGenerator.generateTrajectory(
                leftNotePose, 
                List.of(), 
                speakerPoseWithMargin, // robotStartPose
                config);

        // TODO SequentialCommandGroup that starts intake, goes to note, stops intake, returns to speaker, then shoots
        addCommands(
            new InstantCommand(() -> swerve.resetOdometry(robotStartPose)),
            new WaitCommand(2), 
            new InstantCommand(() -> swerve.stopModules()),
 
            // first and second notes
            new SetShooterCommand(shooter, intake) // replace with shoot command
            // intake.setIntakeCommand(1), // replace with start intake command
            // new DriveTrajectory(swerve, goToMiddleNote), 
            // new ReverseNoteCommand(shooter, intake), 
            // new ReverseNoteCommand(shooter, intake), // replace with stop intake command
            // new DriveTrajectory(swerve, goToSpeakerFromMiddleNote), 
            // new SetShooterCommand(shooter, intake), // replace with shoot command
            // new DriveTrajectory(swerve, goToMiddleNote)
            
            // third (left) note  TODO: maybe remove for some matches
            // intake.setIntakeCommand(1), // replace with start intake command
            // new DriveTrajectory(swerve, goToLeftNote), 
            // intake.setIntakeCommand(0), // replace with stop intake command
            // new DriveTrajectory(swerve, goToSpeakerFromLeftNote), 
            // new SetShooterCommand(shooter, intake) // replace with shoot command
            );
    }
}
