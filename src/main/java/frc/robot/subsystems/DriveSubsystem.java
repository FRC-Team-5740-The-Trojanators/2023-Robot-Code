// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANBusIDs;
import frc.robot.Constants.SwerveDriveModuleConstants;
import lib.LazyTalonFX;


public class DriveSubsystem extends SubsystemBase
{
  public PigeonIMU m_imu = new PigeonIMU(CANBusIDs.k_pigeonID);
  DriveSubsystem m_drivetrain;

  private SwerveModuleState[] m_states = new SwerveModuleState[]
  {
      new SwerveModuleState(0.0, new Rotation2d(0)),
      new SwerveModuleState(0.0, new Rotation2d(0)),
      new SwerveModuleState(0.0, new Rotation2d(0)),
      new SwerveModuleState(0.0, new Rotation2d(0))
  }; 

  public SwerveModule[] modules = new SwerveModule[]
  {
      new SwerveModule(new LazyTalonFX(CANBusIDs.k_LeftFront_DriveMotor), new LazyTalonFX(CANBusIDs.k_LeftFront_SteeringMotor), new CANCoder(CANBusIDs.leftFrontCANCoderId), Rotation2d.fromDegrees(SwerveDriveModuleConstants.leftFrontOffset)), // Left Front
      new SwerveModule(new LazyTalonFX(CANBusIDs.k_RightFront_DriveMotor), new LazyTalonFX(CANBusIDs.k_RightFront_SteeringMotor), new CANCoder(CANBusIDs.rightFrontCANCoderId), Rotation2d.fromDegrees(SwerveDriveModuleConstants.rightFrontOffset)), // Right Front
      new SwerveModule(new LazyTalonFX(CANBusIDs.k_LeftRear_DriveMotor), new LazyTalonFX(CANBusIDs.k_LeftRear_SteeringMotor), new CANCoder(CANBusIDs.leftRearCANCoderId), Rotation2d.fromDegrees(SwerveDriveModuleConstants.leftRearOffset)), // Left Rear
      new SwerveModule(new LazyTalonFX(CANBusIDs.k_RightRear_DriveMotor), new LazyTalonFX(CANBusIDs.k_RightRear_SteeringMotor), new CANCoder(CANBusIDs.rightRearCANCoderId), Rotation2d.fromDegrees(SwerveDriveModuleConstants.rightRearOffset)), // Right Rear
  };

  private SwerveModulePosition[] swerveModulePosition = new SwerveModulePosition[]
  {
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition()
  };
   
 public DriveSubsystem(boolean calibrateGyro) 
  {
    if(calibrateGyro) 
    {
      m_imu.setYaw(0); //recalibrates gyro offset
    }
        
    for(int i = 0; i < 4; i++)
    {
       modules[i].resetDriveEncoder();
    }

    m_imu.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_1_General, 20);
  }

  public void setSwerveModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, SwerveDriveModuleConstants.k_MaxTeleSpeed);
        modules[0].setDesiredState(desiredStates[0]);
        modules[1].setDesiredState(desiredStates[1]);
        modules[2].setDesiredState(desiredStates[2]);
        modules[3].setDesiredState(desiredStates[3]);
  }

  public void teleDrive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) 
  {       
    m_states =
      SwerveDriveModuleConstants.kinematics.toSwerveModuleStates(
          fieldRelative
              ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getHeading(false))
              : new ChassisSpeeds(xSpeed, ySpeed, rot));
      //SwerveDriveKinematics.desaturateWheelSpeeds(m_states, SwerveDriveModuleConstants.k_MaxTeleSpeed);
      setSwerveModuleStates(m_states);
  }
    
  public void resetEncoders()
  {
    modules[0].resetDriveEncoder();
    modules[1].resetDriveEncoder();
    modules[2].resetDriveEncoder();
    modules[3].resetDriveEncoder();
  }

  public void resetIMU()
  {
    m_imu.setYaw(0);
  }

  private final SwerveDriveOdometry m_odometry = 
      new SwerveDriveOdometry(SwerveDriveModuleConstants.kinematics, getHeading(true), swerveModulePosition);

  public Pose2d getPose() 
  {
      SmartDashboard.putString("pose", m_odometry.getPoseMeters().toString());
      return m_odometry.getPoseMeters();
  }

  public Rotation2d getHeading(boolean positive)
  { if(positive)
    {
      return Rotation2d.fromDegrees(m_imu.getYaw());
    }
    else
    {
      return Rotation2d.fromDegrees(-m_imu.getYaw());
    }
  }

  public Rotation2d getPitch(boolean positive)
  { if(positive)
    {
      return Rotation2d.fromDegrees(m_imu.getPitch());
    }
    else
    {
      return Rotation2d.fromDegrees(-m_imu.getPitch());
    }
  }

  public Rotation2d getRoll(boolean positive)
  { if(positive)
    {
      return Rotation2d.fromDegrees(m_imu.getRoll());
    }
    else
    {
      return Rotation2d.fromDegrees(-m_imu.getRoll());
    }
  }

  public void resetOdometry(Pose2d pose) 
  {
    m_odometry.resetPosition(
      getHeading(true),
        new SwerveModulePosition[] {
          modules[0].getPosition(),
          modules[1].getPosition(),
          modules[2].getPosition(),
          modules[3].getPosition()
        },
        pose);
  }

  @Override
  public void periodic() 
  {
    m_odometry.update(
      getHeading(true),
      new SwerveModulePosition[] 
      {
        modules[0].getPosition(),
        modules[1].getPosition(),
        modules[2].getPosition(),
        modules[3].getPosition()
      });
      SmartDashboard.putString("Pitch", getPitch(true).toString());
      SmartDashboard.putString("Roll", getRoll(true).toString());
      SmartDashboard.putString("Position 0", modules[0].getPosition().toString());
      SmartDashboard.putString("Position 1", modules[1].getPosition().toString());
      SmartDashboard.putString("Position 2", modules[2].getPosition().toString());
      SmartDashboard.putString("Position 3", modules[3].getPosition().toString());

      SmartDashboard.putString("Vel 0", modules[0].getState().toString());
      SmartDashboard.putString("Vel 1", modules[1].getState().toString());
      SmartDashboard.putString("Vel 2", modules[2].getState().toString());
      SmartDashboard.putString("Vel 3", modules[3].getState().toString());
  }
}