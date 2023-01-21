// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionTargeting;

public class TargetCommand extends CommandBase 
{
    /** Creates a new TargetCommand. */
  DriveSubsystem m_drivetrain;  
  boolean m_isFinished;
  int m_pipeline;
  VisionTargeting m_visionTargeting; 

  public TargetCommand(DriveSubsystem drivetrain, VisionTargeting visionTargeting, int pipeline) //pass in pipeline of target
  {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    m_pipeline = pipeline;
    m_visionTargeting = visionTargeting;
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    //set correct camera pipeline
    if(m_pipeline == 0)
    {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
    }
    if(m_pipeline == 1)
    {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
    }
   if(m_pipeline == 2)
    {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(2);
    }
      SmartDashboard.putBoolean("Target Sees", false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if(m_visionTargeting.seesTarget())
    {
      SmartDashboard.putBoolean("Target Sees",true);
      if(m_visionTargeting.getLimelightTX() < 0)
      {
        m_drivetrain.teleDrive(0, 0, -1, true);
      } 
      else
        if(m_visionTargeting.getLimelightTX() > 0)
        {
          m_drivetrain.teleDrive(0, 0, 1, true);
        } 
        else 
        {
          m_drivetrain.teleDrive(0, 0, 0, true);
        }
    }
    else 
    {

    }
    //use tx, ty, ta values to rotate drivetrain
    //do some math...
    //drive(arguments)...
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    //m_isFinished = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    //TODO set pipeline back to default
    return m_isFinished;
  }
}
