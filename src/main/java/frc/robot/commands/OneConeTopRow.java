// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveDriveModuleConstants;
import frc.robot.subsystems.DriveSubsystem;

public class OneConeTopRow extends CommandBase 
{
  private final DriveSubsystem m_driveSubsystem; 
  private SwerveAutoBuilder m_autoBuilder; 
  private String m_path;
  private double m_maxVelocity;
  private double m_maxAcceleration;
  private List<PathPlannerTrajectory> pathGroup; 
  /** Creates a new OneConeTopRow. */
  public OneConeTopRow(DriveSubsystem driveSubsystem, String path, double maxVelocity, double maxAcceleration) 
  {
    m_driveSubsystem = driveSubsystem;
    m_path = path;
    m_maxVelocity = maxVelocity;
    m_maxAcceleration = maxAcceleration;

    addRequirements(driveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
       // This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
// for every path in the group
     //List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("FullAuto", new PathConstraints(1, 1));
     pathGroup = PathPlanner.loadPathGroup("FullAuto", new PathConstraints(4, 3));

// This is just an example event map. It would be better to have a constant, global event map
// in your code that will be used by all path following commands.
     HashMap<String, Command> eventMap = new HashMap<>();
  //eventMap.put("marker1", new PrintCommand("Passed marker 1"));
   //eventMap.put("intakeDown", new IntakeDown());

// Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
    m_autoBuilder = new SwerveAutoBuilder(
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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return true;
    //m_autoBuilder.fullAuto(pathGroup);;
  }
}
