// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivebaseConstants;

public class SwerveSubsystem extends SubsystemBase {
    double fl = -0.16873788666742;
    private final SwerveModule frontLeft = new SwerveModule(1, false, true, 1.347+2.988, false);
    private final SwerveModule frontRight = new SwerveModule(2, true, true, 0.688, false);
    private final SwerveModule backRight = new SwerveModule(3, true, true, 1.977+0.16, false);
    private final SwerveModule backLeft = new SwerveModule(4, true, true, 0.759+0.08, false);

    private final AHRS gyro = new AHRS(Port.kUSB1, SerialDataType.kProcessedData, Byte.parseByte("100"));

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
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Robot Heading", getHeading());  // -frontLeft.getAbsoluteEncoderRad()
        SmartDashboard.putNumber("Swerve["+frontLeft.moduleID+"] Absolute Encoder Val (RAD)", frontLeft.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Swerve["+frontRight.moduleID+"] Absolute Encoder Val (RAD)", frontRight.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Swerve["+backRight.moduleID+"] Absolute Encoder Val (RAD)", backRight.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Swerve["+backLeft.moduleID+"] Absolute Encoder Val (RAD)", backLeft.getAbsoluteEncoderRad());

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
}
