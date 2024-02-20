// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SetShooterCommand;
import frc.robot.commands.SwerveCmdJoystick;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  /*private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(14);
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(15);
  */
  // private final PneumaticSubsystem pneumaticSubsystem = new PneumaticSubsystem(0, 1);

  private final CommandXboxController controller =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  // private CANSparkMax spark1 = new CANSparkMax(15, MotorType.kBrushless);
  // private CANSparkMax spark2 = new CANSparkMax(16, MotorType.kBrushless);
  private ShooterSubsystem shooter = new ShooterSubsystem(15);
  // private IntakeSubsystem intake = new IntakeSubsystem(16);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Set default command
    /*swerveSubsystem.setDefaultCommand(
      new SwerveCmdJoystick(
        swerveSubsystem, 
        () -> -controller.getLeftX(), //positive?
        () -> controller.getLeftY(), 
        () -> controller.getRightX(), 
        () -> true));  // true
      */
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    //controller.leftBumper().onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));

    // TESTS
    boolean testingAngle = true;
    // controller.y().onTrue(new InstantCommand(() -> swerveSubsystem.testModule(1, 0.5, testingAngle)));
    // controller.b().onTrue(new InstantCommand(() -> swerveSubsystem.testModule(2, 0.5, testingAngle)));
    // controller.a().onTrue(new InstantCommand(() -> swerveSubsystem.testModule(3, 0.5, testingAngle)));
    // controller.x().onTrue(new InstantCommand(() -> swerveSubsystem.testModule(4, 0.5, testingAngle)));
    /*
    controller.leftTrigger().onTrue(new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(0.25)));
    controller.leftTrigger().onFalse(new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(0)));
    controller.rightBumper().onTrue(new InstantCommand(() -> swerveSubsystem.stopModules()));
    controller.rightTrigger().onTrue(new InstantCommand(() -> shooterSubsystem.setShooterSpeed(0.5), shooterSubsystem));
    controller.rightTrigger().onFalse(new InstantCommand(() -> shooterSubsystem.setShooterSpeed(0)));
    */
    // controller.a().onTrue(new InstantCommand(() -> pneumaticSubsystem.setSolenoid(), pneumaticSubsystem));
    // controller.b().onTrue(new InstantCommand(() -> pneumaticSubsystem.getPressure(), pneumaticSubsystem));

    // controller.rightBumper().onTrue(intake.setIntakeCommand(0.7));
    // controller.leftBumper().onTrue(shooter.setShooterCommand(1));
    // controller.rightBumper().onFalse(intake.setIntakeCommand(0));
    // controller.leftBumper().onFalse(shooter.setShooterCommand(0));
    // controller.leftBumper().onTrue(new SetShooterCommand(shooter, intake));
    controller.leftBumper().onTrue(new InstantCommand(() -> shooter.setShooterSpeed(1)));
    controller.leftBumper().onFalse(new InstantCommand(() -> shooter.setShooterSpeed(0)));



    // controller.a().onTrue(new InstantCommand(() -> swerveSubsystem.setOffsets()));

    


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}