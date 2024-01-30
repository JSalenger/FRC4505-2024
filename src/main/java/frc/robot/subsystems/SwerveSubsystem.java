// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivebaseConstants;

public class SwerveSubsystem extends SubsystemBase {
    double[] offsets = {0.834, 0.657, 4.139, 0.500};
    private final SwerveModule frontLeft = new SwerveModule(1, true, true, offsets[0], false);
    private final SwerveModule frontRight = new SwerveModule(2, true, true, offsets[1], false);
    private final SwerveModule backRight = new SwerveModule(3, true, true, offsets[2], false);
    private final SwerveModule backLeft = new SwerveModule(4, true, true, offsets[3], false);

    private final AHRS gyro = new AHRS(Port.kUSB1, SerialDataType.kProcessedData, Byte.parseByte("100"));
    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
        DrivebaseConstants.kDriveKinematics, 
        getRotation2d(), getPositions());

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading(); // requires a delay so that the gyroscope can start up & calibrate
            } catch (Exception e) {}    
        }).start();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        // -angle because google told me that it should be counterclockwise positive
        return Math.IEEEremainder(-gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    @Override
    public void periodic() {

        odometry.update(getRotation2d(), getPositions());

        SmartDashboard.putNumber("Robot Heading", getHeading());  // -frontLeft.getAbsoluteEncoderRad()
        SmartDashboard.putNumber("Swerve["+frontLeft.moduleID+"] Absolute Encoder Val (RAD)", frontLeft.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Swerve["+frontRight.moduleID+"] Absolute Encoder Val (RAD)", frontRight.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Swerve["+backRight.moduleID+"] Absolute Encoder Val (RAD)", backRight.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Swerve["+backLeft.moduleID+"] Absolute Encoder Val (RAD)", backLeft.getAbsoluteEncoderRad());

        SmartDashboard.putNumber("Swerve["+frontLeft.moduleID+"] Abs Enc (DEG)", Math.toDegrees(frontLeft.getAbsoluteEncoderRad()));
        SmartDashboard.putNumber("Swerve["+frontRight.moduleID+"] Abs Enc (DEG)", Math.toDegrees(frontRight.getAbsoluteEncoderRad()));
        SmartDashboard.putNumber("Swerve["+backRight.moduleID+"] Abs Enc (DEG)", Math.toDegrees(backRight.getAbsoluteEncoderRad()));
        SmartDashboard.putNumber("Swerve["+backLeft.moduleID+"] Abs Enc (DEG)", Math.toDegrees(backLeft.getAbsoluteEncoderRad()));

        SmartDashboard.putNumber("Swerve["+frontLeft.moduleID+"] Abs Enc Raw (RAD)", frontLeft.getAbsoluteEncoderRadRaw());
        SmartDashboard.putNumber("Swerve["+frontRight.moduleID+"] Abs Enc Raw (RAD)", frontRight.getAbsoluteEncoderRadRaw());
        SmartDashboard.putNumber("Swerve["+backRight.moduleID+"] Abs Enc Raw", backRight.getAbsoluteEncoderRadRaw());
        SmartDashboard.putNumber("Swerve["+backLeft.moduleID+"] Abs Enc Raw (RAD)", backLeft.getAbsoluteEncoderRadRaw());

    } 

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getRotation2d(), getPositions(), pose);
    }

    public SwerveModulePosition[] getPositions() {  // useful for auto
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        positions[0] = frontLeft.getPosition();
        positions[1] = frontRight.getPosition();
        positions[2] = backRight.getPosition();
        positions[3] = backLeft.getPosition();
        return positions;
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModules(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DrivebaseConstants.kMaxSpeedMetersPerSecond); // TODO: MEASURE MAX SPEED
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backRight.setDesiredState(desiredStates[2]);
        backLeft.setDesiredState(desiredStates[3]);
    }

    // FOR MODULE TESTING
    public void testModule(int id, double speed, boolean usingAngleMotor) {
        SwerveModule mod;
        switch(id) {
            case 1: mod = frontLeft; break;
            case 2: mod = frontRight; break;
            case 3: mod = backRight; break;
            case 4: mod = backLeft; break;
            default: mod = frontLeft; break;
        }
        if(usingAngleMotor) {
            mod.setTurningMotor(speed);
        } 
        else {
            mod.setDriveMotor(speed);
        }
    }

    public void setOffsets() {
        frontLeft.setOffset(frontLeft.getAbsoluteEncoderRadRaw()+Math.PI/2);
        frontRight.setOffset(frontRight.getAbsoluteEncoderRadRaw()-Math.PI/2);
        backLeft.setOffset(backLeft.getAbsoluteEncoderRadRaw()+Math.PI/2);
        backRight.setOffset(backRight.getAbsoluteEncoderRadRaw()-Math.PI/2);
    }
}