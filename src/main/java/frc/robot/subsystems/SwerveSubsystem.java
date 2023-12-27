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
    
    private final SwerveModule frontLeft = new SwerveModule(1, false, false, 0, false);
    private final SwerveModule frontRight = new SwerveModule(2, true, true, 0, true);
    private final SwerveModule backRight = new SwerveModule(3, true, true, 0, true);
    private final SwerveModule backLeft = new SwerveModule(4, false, false, 0, false);

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
        SmartDashboard.putNumber("Robot Heading", getHeading());
    } 

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModules(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DrivebaseConstants.kMaxSpeedMetersPerSecond); // TODO: MEASURE MAX SPEED
        //frontLeft.setDesiredState(desiredStates[0]);
        //frontRight.setDesiredState(desiredStates[1]);
        backRight.setDesiredState(desiredStates[2]);
        //backLeft.setDesiredState(desiredStates[3]);
    }
}
