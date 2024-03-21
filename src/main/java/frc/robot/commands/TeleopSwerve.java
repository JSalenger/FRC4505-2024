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

public class TeleopSwerve extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    private final Vision limelight;
    private Supplier<Boolean> autoAimingFunction;
    // private PIDController headingPID;

    public TeleopSwerve(SwerveSubsystem swerveSubsystem, Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpeedFunction, Supplier<Boolean> fieldOrientedFunction, Vision limelight, Supplier<Boolean> autoAimingFunction) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpeedFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.limelight = limelight; // auto aim at apriltag
        this.autoAimingFunction = autoAimingFunction;
        // this.headingPID = new PIDController(1/360, 0, 0);
        this.xLimiter = new SlewRateLimiter(DrivebaseConstants.kTeleMaxDriveAccelerationMetersPerSecond);
        this.yLimiter = new SlewRateLimiter(DrivebaseConstants.kTeleMaxDriveAccelerationMetersPerSecond);
        this.turningLimiter = new SlewRateLimiter(DrivebaseConstants.kTeleMaxAngularAccelerationMetersPerSecond);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        //1. Get real-time joystick vals
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();
        
        if(autoAimingFunction.get()) {
            double autoAimSpeed = -limelight.getTx()/40;
            turningSpeed = autoAimSpeed;
            swerveSubsystem.updateLastHeading();
        } else if(Math.abs(turningSpdFunction.get()) > OIConstants.kDeadband) {
            swerveSubsystem.updateLastHeading();
        } else {
            // if(swerveSubsystem.isMaintainingHeading()) {
            //     turningSpeed = swerveSubsystem.getHeadingCorrection();
            // }
            
        }

        //2. apply deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        if(turningSpeed == 0.0 && swerveSubsystem.isMaintainingHeading()) {
                turningSpeed = swerveSubsystem.getHeadingCorrection();
            }

        //3. make driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * DrivebaseConstants.kTeleMaxDriveSpeed;
        ySpeed = yLimiter.calculate(ySpeed) * DrivebaseConstants.kTeleMaxDriveSpeed;
        turningSpeed = turningLimiter.calculate(turningSpeed) * DrivebaseConstants.kTeleMaxAngularSpeed;
        
        //4. construct desired chassis speeds
        swerveSubsystem.drive(xSpeed, ySpeed, turningSpeed, fieldOrientedFunction.get());

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