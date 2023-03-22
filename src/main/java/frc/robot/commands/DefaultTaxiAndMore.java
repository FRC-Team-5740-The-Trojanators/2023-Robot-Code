// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import frc.robot.Constants.ArmPositionConstants;
import frc.robot.Constants.SwerveDriveModuleConstants;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

public class DefaultTaxiAndMore extends CommandBase
{
  /** Creates a new DefaultTaxiAndMore. */
  private DriveSubsystem m_driveSubsystem;
  private Claw m_claw;
  private Shoulder m_shoulder;
  private Wrist m_wrist;
  private PPSwerveControllerCommand m_swerveController;
  private Timer m_timer = new Timer();
  private PathPlannerTrajectory m_forwardTraj = PathPlanner.loadPath("DefaultTaxiAndMore", new PathConstraints(4, 3));
  private PathPlannerTrajectory m_reverseTraj = PathPlanner.loadPath("DefaultTaxiAndMore2", new PathConstraints(4, 3));
  private State state;

  public DefaultTaxiAndMore(DriveSubsystem driveSubsystem, Claw claw, Shoulder shoulder, Wrist wrist)
  {
    m_driveSubsystem = driveSubsystem;
    m_claw = claw;
    m_shoulder = shoulder;
    m_wrist = wrist;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem, m_claw, m_shoulder, m_wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    m_timer.reset();
    m_timer.start();
    state = State.INIT;
  }

  //private State laststate = State.INIT;

   private enum State
   {
      INIT,
      RAISE_ARM,
      STRAIGHT_FORWARD,
      SCORE,
      REVERSE,
      FINISHED,
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    switch(state)
    {
      case INIT:
        //System.out.println("State " + state);
        m_driveSubsystem.setZeroState();
        m_driveSubsystem.resetOdometry(m_forwardTraj.getInitialHolonomicPose());
        m_claw.forwardClawMotor();

        if (m_timer.get() > 1)
        {
          state = State.RAISE_ARM;
        }
      break;

      case RAISE_ARM:
        //System.out.println("State " + state);
        m_shoulder.setSetpoint(ArmPositionConstants.shoulderTopGridCone);
        m_wrist.setSetpoint(ArmPositionConstants.wristTopGridCone);
        m_claw.forwardClawMotor();

        if (m_timer.get() > 6)
        {
          m_swerveController = new PPSwerveControllerCommand(
            m_forwardTraj,
            m_driveSubsystem::getPose,
            SwerveDriveModuleConstants.k_AutoKinematics, // SwerveDriveKinematics
            new PIDController(SwerveDriveModuleConstants.k_pTransController, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
            new PIDController(SwerveDriveModuleConstants.k_pTransController, 0.0, 0.0),
            new PIDController(SwerveDriveModuleConstants.k_pThetaController, 0.0, 0.0),
            m_driveSubsystem::setSwerveModuleStatesAuto,
            false,
            m_driveSubsystem);

          state = State.STRAIGHT_FORWARD;
          m_swerveController.initialize();
        }
      break;

      case STRAIGHT_FORWARD:
        //System.out.println("State " + state);
        m_shoulder.setSetpoint(ArmPositionConstants.shoulderTopGridCone);
        m_wrist.setSetpoint(ArmPositionConstants.wristTopGridCone);
        m_claw.forwardClawMotor();

        m_swerveController.execute();

        if (m_swerveController.isFinished() && (m_timer.get() > 9))
        {
          state = State.SCORE;
          m_swerveController.end(true);
        }
      break;

      case SCORE:
        //System.out.println("State " + state);
        m_shoulder.setSetpoint(ArmPositionConstants.shoulderTopGridCone);
        m_wrist.setSetpoint(ArmPositionConstants.wristTopGridCone);
        m_claw.reverseClawMotor();

        if (m_timer.get() > 11)
        {
          m_swerveController = new PPSwerveControllerCommand(
            m_reverseTraj,
            m_driveSubsystem::getPose,
            SwerveDriveModuleConstants.k_AutoKinematics, // SwerveDriveKinematics
            new PIDController(SwerveDriveModuleConstants.k_pTransController, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
            new PIDController(SwerveDriveModuleConstants.k_pTransController, 0.0, 0.0),
            new PIDController(SwerveDriveModuleConstants.k_pThetaController, 0.0, 0.0),
            m_driveSubsystem::setSwerveModuleStatesAuto,
            false,
            m_driveSubsystem);

          state = State.REVERSE;
          m_swerveController.initialize();
        }   
      break;

      case REVERSE:
        //System.out.println("State " + state);
        m_shoulder.setSetpoint(ArmPositionConstants.shoulderTopGridCone);
        m_wrist.setSetpoint(ArmPositionConstants.wristTopGridCone);
        m_claw.reverseClawMotor();

        m_swerveController.execute();

          if (m_swerveController.isFinished())
            {
              state = State.FINISHED;
              m_swerveController.end(true);
            }  
        break;

        case FINISHED:
          //System.out.println("State " + state);
          m_shoulder.setSetpoint(ArmPositionConstants.shoulderStowed);
          m_wrist.setSetpoint(ArmPositionConstants.wristStowed);
          m_claw.stopClawMotor();
          m_driveSubsystem.teleDrive(0, 0, 0, true);
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }
}
