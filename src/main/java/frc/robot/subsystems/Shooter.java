package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private CANSparkMax shooterMotor;
    private RelativeEncoder shooterEncoder;

    public Shooter() {
        shooterMotor = new CANSparkMax(20, MotorType.kBrushless);
        shooterEncoder = shooterMotor.getEncoder();
        resetEncoder();
    }

    public void setSpeed(double speed) {
        shooterMotor.set(speed);
    }

    public void resetEncoder() {
        shooterEncoder.setPosition(0);
    }

    public RelativeEncoder getEncoder() {
        return shooterEncoder;
    }
}
