// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.SwerveDriveModuleConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TestAuto extends CommandBase 
{ 
  DriveSubsystem m_drivetrain;  
  PIDController xController;
  PIDController yController;
  /* 
  private SwerveModuleState m_states;
  public void teleDrive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) 
  {       
    m_states =
      SwerveDriveModuleConstants.kinematics.toSwerveModuleStates(
          fieldRelative
              ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getHeading(false))
              : new ChassisSpeeds(xSpeed, ySpeed, rot));
      setSwerveModuleStatesTele(m_states);

  */
  
  /** Creates a new TestAuto. */
  public TestAuto(DriveSubsystem drivetrain) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    {
      m_drivetrain.setZeroState();
      m_drivetrain.resetOdometry( new Pose2d(0, 0, new Rotation2d(0)));
  
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
              List.of(new Translation2d(.5, 0), new Translation2d(1, 0)),
              // End 3 meters straight ahead of where we started, facing forward
              new Pose2d(1.5,0, new Rotation2d(0)),
              config);
  
      var thetaController =
          new ProfiledPIDController(
            SwerveDriveModuleConstants.k_pThetaController, 0, 0, SwerveDriveModuleConstants.k_ThetaControllerConstraints);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);
  
      SmartDashboard.putData("xController", xController);
      SmartDashboard.putData("yController", yController);
     
      SwerveControllerCommand swerveControllerCommand =
          new SwerveControllerCommand(
              exampleTrajectory,
              m_drivetrain::getPose, // Functional interface to feed supplier
              SwerveDriveModuleConstants.kinematics,
  
              // Position controllers
              xController,
              yController,
              thetaController,
              m_drivetrain::setSwerveModuleStatesAuto,
              m_drivetrain);
  
      // Reset odometry to the starting pose of the trajectory.
      m_drivetrain.resetOdometry(exampleTrajectory.getInitialPose());
      
      // Run path following command, then stop at the end.
      
      m_drivetrain.teleDrive(0, 0, 0, false);
      //swerveControllerCommand.andThen(() ->
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {

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
