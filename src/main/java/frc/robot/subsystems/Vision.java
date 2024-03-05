package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private double tx = 0;
    private double ty = 0;
    public Vision() {
    }

    @Override
    public void periodic() {
        tx = table.getEntry("tx").getDouble(0.0);
        ty = table.getEntry("ty").getDouble(0.0);
        SmartDashboard.putNumber("tx", getTx());
        SmartDashboard.putNumber("ty", getTy());
    }

    public double getTx() {
        return tx;
    }

    public double getTy() {
        return ty;
    }
}
