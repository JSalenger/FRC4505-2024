// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax shooterMotor;
  private final RelativeEncoder shooterEncoder;

  public ShooterSubsystem(int shooterId){
    this.shooterMotor = new CANSparkMax(shooterId,MotorType.kBrushless);
    this.shooterEncoder = this.shooterMotor.getEncoder();
    shooterEncoder.setPosition(0);
  }
  
  public void setShooterSpeed(double speed){
    shooterMotor.set(speed);
  }
  
  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
