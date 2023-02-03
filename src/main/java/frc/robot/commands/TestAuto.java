// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TestAuto extends CommandBase {
  /** Creates a new TestAuto. */
  public TestAuto() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
/* 
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
            List.of(new Translation2d(0.5, 0), new Translation2d(1.5, 0)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(2,0, new Rotation2d(0)),
            config);

    var thetaController =
        new ProfiledPIDController(
          SwerveDriveModuleConstants.k_pThetaController, 0, 0, SwerveDriveModuleConstants.k_ThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SmartDashboard.putData("xController", xController);
    SmartDashboard.putData("yController", yController);
    m_driveSubsystem.setZeroState();
    m_driveSubsystem.resetOdometry( new Pose2d(0, 0, new Rotation2d(0)));
    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            m_driveSubsystem::getPose, // Functional interface to feed supplier
            SwerveDriveModuleConstants.kinematics,

            // Position controllers
            xController,
            yController,
            thetaController,
            m_driveSubsystem::setSwerveModuleStatesAuto,
            m_driveSubsystem);

    // Reset odometry to the starting pose of the trajectory.
    m_driveSubsystem.resetOdometry(exampleTrajectory.getInitialPose());
    
    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_driveSubsystem.teleDrive(0, 0, 0, false));
  }
  */