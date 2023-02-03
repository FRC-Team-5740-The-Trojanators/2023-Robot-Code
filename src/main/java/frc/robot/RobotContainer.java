// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.HIDConstants;
import frc.robot.Constants.SwerveDriveModuleConstants;
import frc.robot.commands.Balance;
import frc.robot.commands.DefaultTaxi;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.TargetCommand;
import frc.robot.commands.ZeroSwerveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionTargeting;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final DriveSubsystem m_driveSubsystem = new DriveSubsystem(false);
  public final VisionTargeting m_visionTargeting = new VisionTargeting();
  XboxController m_driverController = new XboxController(HIDConstants.k_DriverControllerPort);

  //private final DefaultTaxi m_autoDefault = new DefaultTaxi(m_driveSubsystem);
  WaitCommand wait1second = new WaitCommand(1);
  private final SequentialCommandGroup auto1 = new SequentialCommandGroup(new ZeroSwerveCommand(m_driveSubsystem), wait1second);
  private final SwerveDriveCommand m_driveCommand = new SwerveDriveCommand(m_driveSubsystem, m_driverController);

  public static JoystickButton coneTarget, cubeTarget, xBalance, yBalance, aprilTag, zeroDrive;
  public static PIDController xController = new PIDController(5, 0, 0);
  public static PIDController yController = new PIDController(5, 0, 0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() 
  {
    // Configure the button bindings
    configureButtonBindings();

    m_driveSubsystem.setDefaultCommand(m_driveCommand);
    m_driveSubsystem.resetIMU();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() 
  {
    coneTarget = new JoystickButton(m_driverController , HIDConstants.kA);
    cubeTarget = new JoystickButton(m_driverController , HIDConstants.kB);
    xBalance = new JoystickButton(m_driverController, HIDConstants.kX);
    yBalance = new JoystickButton(m_driverController, HIDConstants.kY);
    aprilTag = new JoystickButton(m_driverController , HIDConstants.kBack);
    zeroDrive = new JoystickButton(m_driverController, HIDConstants.kLB);
    coneTarget.whileTrue(new TargetCommand(m_driveSubsystem, m_visionTargeting, 0));
    cubeTarget.whileTrue(new TargetCommand(m_driveSubsystem, m_visionTargeting, 1));
    aprilTag.whileTrue(new TargetCommand(m_driveSubsystem, m_visionTargeting, 2));
    xBalance.whileTrue(new Balance(m_driveSubsystem, true));
    yBalance.whileTrue(new Balance(m_driveSubsystem, false));
    zeroDrive.onTrue(new ZeroSwerveCommand(m_driveSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() 
  {
return null;
  }
}
