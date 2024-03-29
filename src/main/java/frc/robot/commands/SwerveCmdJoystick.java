package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveCmdJoystick extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    public SwerveCmdJoystick(SwerveSubsystem swerveSubsystem, Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpeedFunction, Supplier<Boolean> fieldOrientedFunction) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpeedFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
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

        //2. apply deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        //3. make driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * DrivebaseConstants.kTeleMaxDriveSpeed;
        ySpeed = yLimiter.calculate(ySpeed) * DrivebaseConstants.kTeleMaxDriveSpeed;
        turningSpeed = turningLimiter.calculate(turningSpeed) * DrivebaseConstants.kTeleMaxAngularSpeed;

        //4. construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction.get()) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        } else {
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        //5. convert to swerve module states
        SwerveModuleState[] moduleStates = DrivebaseConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        //6. apply
        swerveSubsystem.setModules(moduleStates);

        // for now, try to set all the wheels to repeatable locations
        // SwerveModuleState testState = new SwerveModuleState(.5*0.1, new Rotation2d(0));
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