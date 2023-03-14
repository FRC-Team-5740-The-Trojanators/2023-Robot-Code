// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.HIDConstants;
import frc.robot.Constants.SwerveDriveModuleConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionTargeting;

public class TargetCommand extends CommandBase 
{
    /** Creates a new TargetCommand. */
  private DriveSubsystem m_drivetrain;
  private XboxController m_controller;
  private boolean m_isFinished;
  private int m_pipeline;
  private VisionTargeting m_visionTargeting; 
  private String m_camera;
  private PIDController m_targetController = new PIDController(.1, 0, 0);

  private final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(SwerveDriveModuleConstants.k_XYslewRate);

  public TargetCommand(DriveSubsystem drivetrain, XboxController controller, VisionTargeting visionTargeting, int pipeline, String camera)
  {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    m_controller = controller;
    m_pipeline = pipeline;
    m_visionTargeting = visionTargeting;
    m_camera = camera;
  
    addRequirements(m_drivetrain);
  }

  private double getJoystickWithDeadBand(double stickValue)
  {
      if(stickValue > HIDConstants.kDeadBand || stickValue < -HIDConstants.kDeadBand)
      {
          if(stickValue < 0)
          {
              return stickValue = -Math.pow(stickValue, 2);
          }
          else
          {
              return stickValue = Math.pow(stickValue, 2);
          }
      } 
      else 
      {
          return 0;
      }
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
    final var xSpeed =
    -xspeedLimiter.calculate(getJoystickWithDeadBand(m_controller.getLeftY())
     * SwerveDriveModuleConstants.k_MaxTeleSpeed) * SwerveDriveModuleConstants.k_XYjoystickCoefficient;

    if(m_visionTargeting.seesTarget())
    {
      SmartDashboard.putBoolean("Target Sees", m_visionTargeting.seesTarget());
      m_drivetrain.teleDrive(-xSpeed, m_targetController.calculate(m_visionTargeting.getLimelightTX(), 0), m_drivetrain.turnToAngle(0) * SwerveDriveModuleConstants.k_MaxAngularSpeed * SwerveDriveModuleConstants.k_RotCoefficient, true);
    }   
    else 
    {
      m_drivetrain.teleDrive(0, 0, 0, true);
    }

    //SmartDashboard.putNumber("Gyro%360", (m_drivetrain.getHeading(true).getDegrees() % 360));
    //SmartDashboard.putData(m_rotController);
    //SmartDashboard.putNumber("Rot_error", m_rotController.getPositionError());
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
