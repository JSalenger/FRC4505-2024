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

public class AutoShootParkRight extends SequentialCommandGroup {
    public AutoShootParkRight(SwerveSubsystem swerve, ShooterSubsystem shooter, IntakeSubsystem intake) {
        TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.DrivebaseConstants.kDriveKinematics);
        //             TODO check pos offsets for start pose
        Pose2d robotStartPose = new Pose2d(2.3 - 0.6, 5.65 - 1.17, new Rotation2d(-Math.PI/3));
        Pose2d middleNotePose = new Pose2d(4.3 + 0.5, 5.65, new Rotation2d(0));
        Pose2d leftNotePose = new Pose2d(4.3, 7, new Rotation2d(0));
        Pose2d rightNotePose = new Pose2d(4.3, 4.3, new Rotation2d(0));
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
        
        Trajectory goToRightNote = 
            TrajectoryGenerator.generateTrajectory(
                robotStartPose, 
                // List.of(new Translation2d(3.2, 4.3)),  // left and then fwd 
                // List.of(new Translation2d(2.75, 4.3)),  // 2.75, 4.3
                List.of(),
                rightNotePose, 
                config);

        Trajectory goToSpeakerFromLeftNote = // change this to right note?
            TrajectoryGenerator.generateTrajectory(
                leftNotePose,  // rightNotePose
                List.of(), 
                speakerPoseWithMargin, // robotStartPose
                config);

        addCommands(
            new InstantCommand(() -> swerve.resetOdometry(robotStartPose)), 
            new InstantCommand(() -> swerve.zeroHeading()),
            // first and second notes
            new SetShooterCommand(shooter, intake), // replace with shoot command
            intake.setIntakeCommand(1), // replace with start intake command
            new DriveTrajectory(swerve, goToRightNote), 
            new ReverseNoteCommand(shooter, intake), 
            new InstantCommand(() -> swerve.zeroHeading())
            );
    }
}
