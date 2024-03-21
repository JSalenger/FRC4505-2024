// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkMax intakeMotor;
  private final RelativeEncoder intakeEncoder;

  public IntakeSubsystem(int intakeMotorID){
    this.intakeMotor = new CANSparkMax(intakeMotorID, MotorType.kBrushless);
    this.intakeEncoder = this.intakeMotor.getEncoder();
    intakeEncoder.setPosition(0);
    intakeEncoder.setPositionConversionFactor(1);
    intakeMotor.setInverted(true);


    // intakeMotor.setSmartCurrentLimit(30);
    // deal with high CAN utilization
    // intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
    // intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
    // intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
    }

  public void fixIntakeDirection() {
    intakeMotor.setInverted(true);
    intakeEncoder.setPositionConversionFactor(1);
  }

  public void setIntakeSpeed(double speed){
    intakeMotor.set(speed);
  }

  public Command setIntakeCommand(double speed) {
    return new InstantCommand(() -> setIntakeSpeed(speed), this);
  }

  public RelativeEncoder getEncoder() {
    return intakeEncoder;
  }

  @Override
  public void periodic() {
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}