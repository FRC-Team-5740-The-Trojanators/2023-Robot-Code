// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
import edu.wpi.first.wpilibj2.command.CommandBase;
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

   // This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
// for every path in the group
     //List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("FullAuto", new PathConstraints(1, 1));

// This is just an example event map. It would be better to have a constant, global event map
// in your code that will be used by all path following commands.
     HashMap<String, Command> eventMap = new HashMap<>();
  //eventMap.put("marker1", new PrintCommand("Passed marker 1"));
   //eventMap.put("intakeDown", new IntakeDown());

// Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      m_driveSubsystem::getPose, // Pose2d supplier
      m_driveSubsystem::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
      SwerveDriveModuleConstants.k_AutoKinematics, // SwerveDriveKinematics
      new PIDConstants(SwerveDriveModuleConstants.k_pTransController, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
      new PIDConstants(SwerveDriveModuleConstants.k_pThetaController, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
      m_driveSubsystem::setSwerveModuleStatesAuto, // Module states consumer used to output to the drive subsystem
      eventMap,
      true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
      m_driveSubsystem // The drive subsystem. Used to properly set the requirements of path following commands
    );
  
  //private final DefaultTaxi m_autoDefault = new DefaultTaxi(m_driveSubsystem);
  private final SwerveDriveCommand m_driveCommand = new SwerveDriveCommand(m_driveSubsystem, m_driverController);
  
  public static JoystickButton coneTarget, cubeTarget, xBalance, yBalance, aprilTag, zeroDrive;

  SendableChooser<CommandBase> auto = new SendableChooser<CommandBase>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() 
  {
    configureButtonBindings();

    m_driveSubsystem.setDefaultCommand(m_driveCommand);
    m_driveSubsystem.resetIMU();
  
    configChooser();
  
  }

  private void configChooser()
  {
    //configure the auto command chooser
    // auto.addOption("Position 1", m_position1Sequential);
    // auto.addOption("Position 2", m_position2Sequential);
    // auto.addOption("Position 3", m_position3Sequential);
    // auto.addOption("Position 4", m_position4Sequential);
    auto.addOption("Full Auto", autoBuilder.fullAuto( PathPlanner.loadPathGroup("FullAuto", new PathConstraints(4, 3))));
    auto.addOption("Schwab", autoBuilder.fullAuto( PathPlanner.loadPathGroup("Schwab", new PathConstraints(4, 3))));
   // auto.addOption("2 Ball R", m_twoBallSeq);
    //auto.addOption("2 Ball L", m_twoBallLeftSeq);

    auto.setDefaultOption("Default taxi", autoBuilder.fullAuto( PathPlanner.loadPathGroup("DefaultTaxi", new PathConstraints(4, 3))));

    SmartDashboard.putData(auto);
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
   

    //return autoBuilder.fullAuto(pathGroup);
    return auto.getSelected();
  }
}

