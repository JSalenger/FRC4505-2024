// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.autos.AutoSpeakerBlue;
import frc.robot.autos.AutoSpeakerRed;
import frc.robot.autos.DriveTrajectory;
import frc.robot.autos.ShootAuto;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ReverseNoteCommand;
import frc.robot.commands.SetShooterCommand;
import frc.robot.commands.SwerveCmdJoystick;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SendableChooser<Command> m_Chooser = new SendableChooser<>();

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

private ShooterSubsystem shooter = new ShooterSubsystem(15);
  private IntakeSubsystem intake = new IntakeSubsystem(16);
  private final PneumaticSubsystem pneumaticSubsystem = new PneumaticSubsystem(0, 1);
;

  private final Joystick rightJoystick = new Joystick(0); // 0 is the USB Port to be used as indicated on the Driver Station
  private final Joystick lefJoystick = new Joystick(1);

  private final CommandXboxController controller =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final Vision limelight = new Vision();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Set default command  TODO uncomment
    swerveSubsystem.setDefaultCommand(
      new SwerveCmdJoystick(
        swerveSubsystem, 
        () -> -controller.getLeftY(), // x is forward and y is sideways (wpilib convention)
        () -> -controller.getLeftX(), 
        () -> -controller.getRightX(), 
        () -> true, 
        limelight, 
        () -> controller.b().getAsBoolean()));  // = is auto aiming

    // Configure the trigger bindings
    configureBindings();

    m_Chooser.setDefaultOption("simple shoot", new ShootAuto(swerveSubsystem, shooter, intake));
    m_Chooser.addOption("2 note speaker middle", new AutoSpeakerBlue(swerveSubsystem, shooter, intake));
    SmartDashboard.putData("auto chooser", m_Chooser);

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
    //---------------------------  TODO uncomment
    controller.x().onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
    controller.y().onTrue(new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d())));
    //---------------------------------
    // TESTS
    boolean testingAngle = false;
    // controller.y().onTrue(new InstantCommand(() -> swerveSubsystem.testModule(1, 0.5, testingAngle), swerveSubsystem));
    // controller.b().onTrue(new InstantCommand(() -> swerveSubsystem.testModule(2, 0.5, testingAngle), swerveSubsystem));
    // controller.a().onTrue(new InstantCommand(() -> swerveSubsystem.testModule(3, 0.5, testingAngle), swerveSubsystem));
    // controller.x().onTrue(new InstantCommand(() -> swerveSubsystem.testModule(4, 0.5, testingAngle), swerveSubsystem));
    // controller.a().onFalse(intake.setIntakeCommand(0));
    //------------------------------------------ TODO uncomment
    // controller.rightBumper().onTrue(new InstantCommand(() -> swerveSubsystem.stopModules()));
    controller.leftBumper().onTrue(new SetShooterCommand(shooter, intake).withTimeout(4));
    controller.a().onTrue(intake.setIntakeCommand(1));
    controller.a().onFalse(new ReverseNoteCommand(shooter, intake).withTimeout(2));

    controller.rightBumper().onTrue(shooter.setShooterCommand(-0.5));
    // controller.rightBumper().onFalse(shooter.setShooterCommand(0));
    controller.rightBumper().onFalse(new ReverseNoteCommand(shooter, intake).withTimeout(2));

    controller.povDown().onTrue(new InstantCommand(() -> swerveSubsystem.toggleHeadingMaintainer()));
    controller.povLeft().onTrue(new InstantCommand(() -> swerveSubsystem.toggleDriveMotorIdleModes()));

    controller.povRight().onTrue(intake.setIntakeCommand(-0.5));
    controller.povRight().onFalse(intake.setIntakeCommand(0));
    // ----------------------------------------------------
    // controller.a().onTrue(new InstantCommand(() -> swerveSubsystem.setOffsets()));
    
    controller.povUp().onTrue(new InstantCommand(() -> pneumaticSubsystem.toggleSolenoid(), pneumaticSubsystem));
    // controller.x().onTrue(new InstantCommand(() -> pneumaticSubsystem.getPressure(), pneumaticSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
    return m_Chooser.getSelected();  // TODO change
  }
}