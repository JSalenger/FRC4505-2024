package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SetShooterCommand2 extends SequentialCommandGroup {
    public SetShooterCommand2(ShooterSubsystem shooter, IntakeSubsystem intake) {
        addRequirements(shooter, intake);
        addCommands(
            shooter.setShooterCommand(1),
            new WaitCommand(1),
            intake.setIntakeCommand(0.5),
            new WaitCommand(0.5),
            shooter.setShooterCommand(0),
            intake.setIntakeCommand(0));
    }
}
