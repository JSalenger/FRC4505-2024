package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SetShooterCommand extends Command {
    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    public SetShooterCommand(ShooterSubsystem shooter, IntakeSubsystem intake) {
        this.shooter = shooter;
        this.intake = intake;
        // addRequirements(shooter, intake);
        shooter.getEncoder().setPosition(0);

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setShooterSpeed(1);
        if(shooter.getEncoder().getPosition() > 20) {  // allow shooter to reach full speed
            intake.setIntakeSpeed(0.3);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setShooterSpeed(0);
        shooter.getEncoder().setPosition(0);
        intake.setIntakeSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return shooter.getEncoder().getPosition() > 100;
    }

    
}