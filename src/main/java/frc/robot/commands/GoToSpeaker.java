package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;

public class GoToSpeaker extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    private final Vision limelight;
    // private PIDController headingPID;

    public GoToSpeaker(SwerveSubsystem swerveSubsystem, Vision limelight) {
        this.swerveSubsystem = swerveSubsystem;
        this.limelight = limelight; // auto aim at apriltag
        // this.headingPID = new PIDController(1/360, 0, 0);
        this.xLimiter = new SlewRateLimiter(DrivebaseConstants.kTeleMaxDriveAccelerationMetersPerSecond);
        this.yLimiter = new SlewRateLimiter(DrivebaseConstants.kTeleMaxDriveAccelerationMetersPerSecond);
        this.turningLimiter = new SlewRateLimiter(DrivebaseConstants.kTeleMaxAngularAccelerationMetersPerSecond);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        //1. Get real-time joystick vals
        double xSpeed = 0;
        double ySpeed = 0;
        double turningSpeed = 0;
        
        double autoAimSpeed = -limelight.getTx()/40;
        turningSpeed = autoAimSpeed;
        swerveSubsystem.updateLastHeading();

        if(limelight.getTx() != 0.0) {
            xSpeed = -0.2;
            ySpeed = limelight.getTx()/20;
        }

        //2. apply deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        //3. make driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * DrivebaseConstants.kTeleMaxDriveSpeed;
        ySpeed = yLimiter.calculate(ySpeed) * DrivebaseConstants.kTeleMaxDriveSpeed;
        turningSpeed = turningLimiter.calculate(turningSpeed) * DrivebaseConstants.kTeleMaxAngularSpeed;
        
        //4. construct desired chassis speeds
        swerveSubsystem.drive(xSpeed, ySpeed, turningSpeed, false);

        // test mode
        // SwerveModuleState testState = new SwerveModuleState(.5*1, new Rotation2d(0));
        // SwerveModuleState[] testModuleStates = {testState, testState, testState, testState};
        // swerveSubsystem.setModules(testModuleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}