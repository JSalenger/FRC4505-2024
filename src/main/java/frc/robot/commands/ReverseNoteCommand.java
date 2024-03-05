package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ReverseNoteCommand extends Command {
    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    public ReverseNoteCommand(ShooterSubsystem shooter, IntakeSubsystem intake) {
        this.shooter = shooter;
        this.intake = intake;
        // addRequirements(shooter, intake);
        shooter.getEncoder().setPosition(0);

        addRequirements(shooter, intake);
    }

    @Override
    public void execute() {
        shooter.fixShooterDirection();
        intake.fixIntakeDirection();  // prevent randomly reversing direction mid-match

        shooter.setShooterSpeed(-0.2);
        intake.setIntakeSpeed(-0.2);
        
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setShooterSpeed(0);
        shooter.getEncoder().setPosition(0);
        intake.setIntakeSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(shooter.getEncoder().getPosition()) > 5;
    }

    
}