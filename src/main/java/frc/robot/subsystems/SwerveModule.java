package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.SwerveModuleConstants;

public class SwerveModule extends SubsystemBase {

    // CAN IDs are assigned starting in the top right corner of the robot moving clockwise
    // Odd numbered IDs are drive motors and Even numbered IDs are turning motors
    // Absolute Encoder ID is assigned on a second pass clockwise around the robot
    // For example, module 2 has driveMotor ID 3, turningMotor ID 4, absoluteEncoder ID 10 
    // Each module has 2 motors so encoder IDs start at 9
    private final int moduleID;

    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;

    private final CANCoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int moduleID, int driveMotorID, boolean driveMotorReversed, int turningMotorID, boolean turningMotorReversed, int CANcoderID, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
        this.moduleID = moduleID;

        this.driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        this.turningMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);
        this.driveMotor.setInverted(driveMotorReversed);
        this.turningMotor.setInverted(turningMotorReversed);
        
        this.driveEncoder = this.driveMotor.getEncoder();
        this.driveEncoder.setPositionConversionFactor(SwerveModuleConstants.kDriveEncoderRot2Meter); // TODO: MEASURE THESE FACTORS
        this.driveEncoder.setVelocityConversionFactor(SwerveModuleConstants.kDriveEncoderRPM2MeterPerSec);
        this.turningEncoder = this.turningMotor.getEncoder();
        this.turningEncoder.setPositionConversionFactor(SwerveModuleConstants.kTurningEncoderRot2Rad);
        this.turningEncoder.setVelocityConversionFactor(SwerveModuleConstants.kTurningEncoderRPM2RadPerSec);


        this.turningPidController = new PIDController(SwerveModuleConstants.kPTurning * .8, 0, 0);
        this.turningPidController.enableContinuousInput(-Math.PI, Math.PI); // Use PI rad as our reference point here because of the conversion factors set above

        this.absoluteEncoder = new CANCoder(CANcoderID);
        this.absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360); // I *think* this is correct
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;

        resetEncoders();
    }

    public SwerveModule(int moduleID, boolean driveMotorReversed, boolean turningMotorReversed, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
        // Infer IDs using schema defined in comment at top of file
        this(moduleID, 2 * (moduleID - 1) + 1, driveMotorReversed, 2 * (moduleID - 1) + 2, turningMotorReversed, 8 + moduleID, absoluteEncoderOffset, absoluteEncoderReversed);
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getAbsolutePosition();
        angle /= 180;
        angle *= Math.PI; // convert to Radians
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < .001) {
            stop();
            return;
        }
        
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DrivebaseConstants.kMaxSpeedMetersPerSecond); // 4 is max speed in m/s TODO: MEASURE MAX SPEED
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve["+this.moduleID+"] State", state.toString());
        SmartDashboard.putNumber("Swerve["+this.moduleID+"] Absolute Encoder Val (RAD)", getAbsoluteEncoderRad());
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}