package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShootCommand extends Command {
    private Shooter shooter;
    public ShootCommand(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setSpeed(0.5);
    }

    @Override
    public boolean isFinished() {
        return shooter.getEncoder().getPosition() > 50;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setSpeed(0);
    }
}
