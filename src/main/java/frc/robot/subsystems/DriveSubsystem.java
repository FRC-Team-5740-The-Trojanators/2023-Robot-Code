// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANBusIDs;
import frc.robot.Constants.SwerveDriveModuleConstants;


public class DriveSubsystem extends SubsystemBase
{
  public Pigeon2 m_imu = new Pigeon2(CANBusIDs.k_pigeon2ID, "CANivorous_Rex");
  DriveSubsystem m_drivetrain;
  Field2d m_field2d = new Field2d();

  private SwerveModuleState[] m_states = new SwerveModuleState[]
  {
      new SwerveModuleState(0.0, new Rotation2d(0)),
      new SwerveModuleState(0.0, new Rotation2d(0)),
      new SwerveModuleState(0.0, new Rotation2d(0)),
      new SwerveModuleState(0.0, new Rotation2d(0))
  }; 

  public SwerveModule[] modules = new SwerveModule[]
  {
      new SwerveModule(new TalonFX(CANBusIDs.k_LeftFront_DriveMotor,"CANivorous_Rex"), new TalonFX(CANBusIDs.k_LeftFront_SteeringMotor,"CANivorous_Rex"), new CANCoder(CANBusIDs.leftFrontCANCoderId, "CANivorous_Rex"), Rotation2d.fromDegrees(SwerveDriveModuleConstants.leftFrontOffset)), // Left Front
      new SwerveModule(new TalonFX(CANBusIDs.k_RightFront_DriveMotor, "CANivorous_Rex"), new TalonFX(CANBusIDs.k_RightFront_SteeringMotor, "CANivorous_Rex"), new CANCoder(CANBusIDs.rightFrontCANCoderId, "CANivorous_Rex"), Rotation2d.fromDegrees(SwerveDriveModuleConstants.rightFrontOffset)), // Right Front
      new SwerveModule(new TalonFX(CANBusIDs.k_LeftRear_DriveMotor, "CANivorous_Rex"), new TalonFX(CANBusIDs.k_LeftRear_SteeringMotor, "CANivorous_Rex"), new CANCoder(CANBusIDs.leftRearCANCoderId, "CANivorous_Rex"), Rotation2d.fromDegrees(SwerveDriveModuleConstants.leftRearOffset)), // Left Rear
      new SwerveModule(new TalonFX(CANBusIDs.k_RightRear_DriveMotor, "CANivorous_Rex"), new TalonFX(CANBusIDs.k_RightRear_SteeringMotor, "CANivorous_Rex"), new CANCoder(CANBusIDs.rightRearCANCoderId, "CANivorous_Rex"), Rotation2d.fromDegrees(SwerveDriveModuleConstants.rightRearOffset)), // Right Rear
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

  public void setZeroState()
  {
    modules[0].setZeroState();
    modules[1].setZeroState();
    modules[2].setZeroState();
    modules[3].setZeroState();
  }
    
  public void setSwerveModuleStatesTele(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, SwerveDriveModuleConstants.k_MaxTeleSpeed);
        modules[0].setDesiredState(desiredStates[0], true);
        modules[1].setDesiredState(desiredStates[1], true);
        modules[2].setDesiredState(desiredStates[2], true);
        modules[3].setDesiredState(desiredStates[3], true);
  }

  public void setSwerveModuleStatesAuto(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, SwerveDriveModuleConstants.k_MaxAutoSpeed);
        modules[0].setDesiredState(desiredStates[0], false);
        modules[1].setDesiredState(desiredStates[1], false);
        modules[2].setDesiredState(desiredStates[2], false);
        modules[3].setDesiredState(desiredStates[3], false);
  }

  public void teleDrive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) 
  {       
    m_states =
      SwerveDriveModuleConstants.k_AutoKinematics.toSwerveModuleStates(
          fieldRelative
              ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getHeading(false))
              : new ChassisSpeeds(xSpeed, ySpeed, rot));
      setSwerveModuleStatesTele(m_states);
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
      new SwerveDriveOdometry(SwerveDriveModuleConstants.k_AutoKinematics, getHeading(true), swerveModulePosition);

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
      m_field2d.setRobotPose(getPose());
      /*SmartDashboard.putString("Pitch", getPitch(true).toString());
      SmartDashboard.putString("Roll", getRoll(true).toString());
      SmartDashboard.putString("Position 0", modules[0].getPosition().toString());
      SmartDashboard.putString("Position 1", modules[1].getPosition().toString());
      SmartDashboard.putString("Position 2", modules[2].getPosition().toString());
      SmartDashboard.putString("Position 3", modules[3].getPosition().toString());
      SmartDashboard.putNumber("encoder 3", modules[3].getDriveEncoder());
      SmartDashboard.putString("Vel 0", modules[0].getState().toString());
      SmartDashboard.putString("Vel 1", modules[1].getState().toString());
      SmartDashboard.putString("Vel 2", modules[2].getState().toString());
      SmartDashboard.putString("Vel 3", modules[3].getState().toString());
      SmartDashboard.putData("field2d", m_field2d);*/
  }
}