// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//We use double solenoids

package frc.robot.subsystems;

import java.io.Console;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticSubsystem extends SubsystemBase {
  private final DoubleSolenoid dSolenoid1;
  private final Compressor compressor;
  /** Creates a new ExampleSubsystem. */
  public PneumaticSubsystem(int fChannel1, int rChannel1) {
    this.dSolenoid1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, fChannel1, rChannel1);
    dSolenoid1.set(Value.kReverse); //Set Double Soleniod to postition B
    compressor = new Compressor(PneumaticsModuleType.REVPH);
  }

  public void toggleSolenoid(){
    dSolenoid1.toggle();
    SmartDashboard.putNumber("solenoid toggle", System.currentTimeMillis());

  }

  public void getPressureSwitchValue(){
    // System.out.println(compressor.getPressureSwitchValue());
    SmartDashboard.putBoolean("pressureSwitch", compressor.getPressureSwitchValue());
  }

  public void getPressure(){
    // System.out.println(compressor.getPressureSwitchValue());
    SmartDashboard.putNumber("pressure", compressor.getPressure());
  }

  @Override
  public void periodic() {
    getPressureSwitchValue();
    getPressure();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}