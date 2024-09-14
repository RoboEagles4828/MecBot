// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DigitalIDs;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.drive.DriveTrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import edu.wpi.first.math.MathUtil;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final DriveTrain m_driveTrain = new DriveTrain();
  private final Shooter m_shooter = new Shooter();
  private final Intake m_intake = new Intake();

  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  private final DigitalInput beamBreak = new DigitalInput(DigitalIDs.kBeamBreak);

  private final SendableChooser<Supplier<Command>> m_autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and command
   * bindings.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    populateAutoChooser();
    SmartDashboard.putData("Auto choices", m_autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
   * constructor with an arbitrary predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_driveTrain.setDefaultCommand(
        m_driveTrain.getDriveCommand(
            () -> -MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.kDriverControllerDeadband),
            () -> MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.kDriverControllerDeadband),
            () -> MathUtil.applyDeadband(m_driverController.getRightX(), OperatorConstants.kDriverControllerDeadband)));

    

    /*
     * Resetting the gyro should be rare, difficult to do accidently, and the robot
     * must be stationary. Use these two buttons pressed at the same time to help
     * accomplish these goals.
     */
    m_driverController.povDown().and(m_driverController.a())
        .onTrue(m_driveTrain.getResetGyroCommand());

    /*
     * As long the left bumper is held, drive robot relative.
     */
    m_driverController.leftBumper()
        .onTrue(Commands.runOnce(() -> m_driveTrain.setFieldRelative(false)))
        .onFalse(Commands.runOnce(() -> m_driveTrain.setFieldRelative(true)));

    m_driverController.leftTrigger()
        .onTrue(
          m_shooter.getIntakeCommand().alongWith(m_intake.getIntakeCommand())
            .until(beamBreak::get)
            .andThen(
              m_shooter.getStopCommand().alongWith(m_intake.getStopCommand())
            )
        ).onFalse(
          m_shooter.getStopCommand().alongWith(m_intake.getStopCommand())
        );
    
    m_driverController.rightTrigger()
        .onTrue(
          m_shooter.getShootCommand().alongWith(m_intake.getShootCommand())
        ).onFalse(
          m_shooter.getStopCommand().alongWith(m_intake.getStopCommand())
        );
  }

  /**
   * Adds all our automode suppliers to the dashboard chooser.
   */
  private void populateAutoChooser() {
    m_autoChooser.setDefaultOption(
        "Do Nothing",
        () -> Autos.doNothingAuto(m_driveTrain));
    m_autoChooser.addOption(
        "Simple Leave",
        () -> Autos.simpleLeaveAuto(m_driveTrain));
    m_autoChooser.addOption(
        "Source Side Score And Leave",
        () -> Autos.sourceSideScoreAndLeave(m_driveTrain));
    m_autoChooser.addOption(
        "Amp Side Score And Leave",
        () -> Autos.ampSideScoreAndLeave(m_driveTrain));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected().get();
  }
}
