// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax shooterMotor;
  private final RelativeEncoder shooterEncoder;

  public ShooterSubsystem(int shooterId){
    this.shooterMotor = new CANSparkMax(shooterId,MotorType.kBrushless);
    this.shooterEncoder = this.shooterMotor.getEncoder();
    shooterEncoder.setPosition(0);
    shooterMotor.setIdleMode(IdleMode.kCoast);
    shooterMotor.setInverted(true);  // check

    // deal with high CAN bus utilization
    shooterMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
    shooterMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
    shooterMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
  }

  public void fixShooterDirection() {
    shooterMotor.setInverted(true);
  }
  
  public void setShooterSpeed(double speed){
    shooterMotor.set(speed);
  }

  public Command setShooterCommand(double speed) {
    return new InstantCommand(() -> setShooterSpeed(speed), this);
  }

  public RelativeEncoder getEncoder() {
    return shooterEncoder;
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("shooter encoder", shooterEncoder.getPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
