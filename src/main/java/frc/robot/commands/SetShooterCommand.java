package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SetShooterCommand extends Command {
    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    private final SlewRateLimiter accLimiter = new SlewRateLimiter(0.05);
    public SetShooterCommand(ShooterSubsystem shooter, IntakeSubsystem intake) {
        this.shooter = shooter;
        this.intake = intake;

        shooter.getEncoder().setPosition(0);

        shooter.fixShooterDirection();
        intake.fixIntakeDirection();  // prevent randomly reversing direction mid-match

        addRequirements(shooter, intake);
        // withTimeout(6);
    }

    @Override
    public void initialize() {
        shooter.getEncoder().setPosition(0);

        shooter.fixShooterDirection();
        intake.fixIntakeDirection();
    }

    @Override
    public void execute() {
        // shooter.fixShooterDirection();
        // intake.fixIntakeDirection();  // prevent randomly reversing direction mid-match

        // shooter.setShooterSpeed(1);
        shooter.setShooterSpeed(0.8);
        // if(Math.abs(shooter.getEncoder().getVelocity()) > 0.75) {
        //     intake.setIntakeCommand(0.5);
        // }
        if(Math.abs(shooter.getEncoder().getPosition()) > 20) {  // allow shooter to reach full speed
            intake.setIntakeSpeed(0.5);
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
        return Math.abs(shooter.getEncoder().getPosition()) > 45; //100
    }

    
}