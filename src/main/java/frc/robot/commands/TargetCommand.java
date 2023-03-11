// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionTargeting;

public class TargetCommand extends CommandBase 
{
    /** Creates a new TargetCommand. */
  private DriveSubsystem m_drivetrain;  
  private boolean m_isFinished;
  private int m_pipeline;
  private VisionTargeting m_visionTargeting; 
  private String m_camera;
  private PIDController m_targetController = new PIDController(.1, 0, 0);
  private PIDController m_rotController = new PIDController(.1, 0, 0);

  public TargetCommand(DriveSubsystem drivetrain, VisionTargeting visionTargeting, int pipeline, String camera)
  {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    m_pipeline = pipeline;
    m_visionTargeting = visionTargeting;
    m_camera = camera;
  
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    //set correct camera pipeline
    NetworkTableInstance.getDefault().getTable(m_camera).getEntry("pipeline").setNumber(m_pipeline);
    SmartDashboard.putBoolean("Target Sees", false);
    m_isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if(m_visionTargeting.seesTarget())
    {
      SmartDashboard.putBoolean("Target Sees",true);
      m_drivetrain.teleDrive(0, m_targetController.calculate(m_visionTargeting.getLimelightTX(), 0), m_rotController.calculate(m_drivetrain.getRawYaw(), 180), false);
    }   
    else 
    {
      m_drivetrain.teleDrive(0, 0, 0, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_isFinished = true;
    NetworkTableInstance.getDefault().getTable("limelight-b").getEntry("pipeline").setNumber(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    //TODO set pipeline back to default
    return m_isFinished;
  }
}
