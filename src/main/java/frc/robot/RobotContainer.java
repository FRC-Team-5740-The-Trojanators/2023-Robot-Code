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
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionTargeting;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
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

  private final DefaultTaxi m_autoDefault = new DefaultTaxi(m_driveSubsystem);
  private final SwerveDriveCommand m_driveCommand = new SwerveDriveCommand(m_driveSubsystem, m_driverController);

  public static JoystickButton coneTarget, cubeTarget, xBalance, yBalance, aprilTag;

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
    coneTarget.whileTrue(new TargetCommand(m_driveSubsystem, m_visionTargeting, 0));
    cubeTarget.whileTrue(new TargetCommand(m_driveSubsystem, m_visionTargeting, 1));
    aprilTag.whileTrue(new TargetCommand(m_driveSubsystem, m_visionTargeting, 2));
    xBalance.whileTrue(new Balance(m_driveSubsystem, true));
    yBalance.whileTrue(new Balance(m_driveSubsystem, false));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() 
  {
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
          SwerveDriveModuleConstants.k_MaxAutoSpeed,
          SwerveDriveModuleConstants.k_MaxAcceleration)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(SwerveDriveModuleConstants.kinematics);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 0), new Translation2d(1.5, 0)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(2, 0, new Rotation2d(0)),
            config);

    var thetaController =
        new ProfiledPIDController(
          SwerveDriveModuleConstants.k_pThetaController, 0, 0, SwerveDriveModuleConstants.k_ThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            m_driveSubsystem::getPose, // Functional interface to feed supplier
            SwerveDriveModuleConstants.kinematics,

            // Position controllers
            new PIDController(3, 0, 0),
            new PIDController(3, 0, 0),
            thetaController,
            m_driveSubsystem::setSwerveModuleStates,
            m_driveSubsystem);

    // Reset odometry to the starting pose of the trajectory.
    m_driveSubsystem.resetOdometry(exampleTrajectory.getInitialPose());
    
    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_driveSubsystem.teleDrive(0, 0, 0, false));
  }
}
