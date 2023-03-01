// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shoulder extends SubsystemBase 
{
  /** Creates a new Shoulder. */
  private double m_lastSpeed = 0;
  private double m_lastTime = Timer.getFPGATimestamp();

  private final TrapezoidProfile.Constraints m_trapConstraints = new TrapezoidProfile.Constraints(1, 0.1);
  private CANSparkMax m_shoulderMotor = new CANSparkMax(Constants.CANBusIDs.k_shoulderMotorID, MotorType.kBrushless);
  private ArmFeedforward m_armFeedforward = new ArmFeedforward(0, .28112, 10.36);
  private ProfiledPIDController m_shoulderMotorPID = new ProfiledPIDController(Constants.MotorPIDValues.k_shoulderMotorP, Constants.MotorPIDValues.k_shoulderMotorI, Constants.MotorPIDValues.k_shoulderMotorD, m_trapConstraints);
  //private PIDController m_shoulderMotorPID = new PIDController(Constants.MotorPIDValues.k_shoulderMotorP, Constants.MotorPIDValues.k_shoulderMotorI, Constants.MotorPIDValues.k_shoulderMotorD);
  private DutyCycleEncoder m_shoulderEncoder = new DutyCycleEncoder(Constants.DigitalInputPort.k_shoulderEncoderPort);

  public Shoulder()
  {
    m_shoulderMotor.restoreFactoryDefaults();
    m_shoulderMotor.enableVoltageCompensation(10);
    m_shoulderMotor.setIdleMode(IdleMode.kBrake);
    m_shoulderMotor.setInverted(false);
    //m_shoulderMotor.setOpenLoopRampRate(1);
    m_shoulderMotorPID.setTolerance(Constants.MotorPIDValues.k_shoulderTolerance);
    m_shoulderMotorPID.disableContinuousInput();
    //m_shoulderMotorPID.reset(getAngleRadians());
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shoulder Abs Encoder", getAbsEncoder());
    SmartDashboard.putNumber("Shoulder Angle Radians", getAngleRadians());
    SmartDashboard.putData(m_shoulderMotorPID);
    SmartDashboard.putNumber("error", m_shoulderMotorPID.getPositionError());
    SmartDashboard.putNumber("Setpoint Pos", m_shoulderMotorPID.getSetpoint().position);
    SmartDashboard.putNumber("Setpoint Vel", m_shoulderMotorPID.getSetpoint().velocity);
  }

  public double setSetpoint(double posSetPoint)
  {
    //return m_shoulderMotorPID.calculate(getAbsEncoder(), posSetPoint)  + m_armFeedforward.calculate(posSetPoint, 0);
    return m_shoulderMotorPID.calculate(getAngleRadians(), posSetPoint) - m_armFeedforward.calculate(posSetPoint, 0);
  }

// Controls a simple motor's position using a SimpleMotorFeedforward
// and a ProfiledPIDm_shoulderMotorPID
  public void goToPosition(double goalPosition) 
  {
    double pidVal = m_shoulderMotorPID.calculate(getAngleRadians(), goalPosition);
    double acceleration = (m_shoulderMotorPID.getSetpoint().velocity - m_lastSpeed) / (Timer.getFPGATimestamp() - m_lastTime);
    m_shoulderMotor.setVoltage(pidVal + m_armFeedforward.calculate(m_shoulderMotorPID.getSetpoint().velocity, acceleration));
    m_lastSpeed = m_shoulderMotorPID.getSetpoint().velocity;
    m_lastTime = Timer.getFPGATimestamp();
  }

  public boolean moveEnd()
  {
    return m_shoulderMotorPID.atSetpoint();
  }

  public void setMotor(double demand)
  {       
    m_shoulderMotor.setVoltage(demand);    
  }

  public void forceMotorExtend()
  {
      m_shoulderMotor.set(.1);
  }

  public void forceMotorRetract()
  {
    m_shoulderMotor.set(-0.1);
  }

  public void forceMotorStop()
  {
    m_shoulderMotor.set(0);
  }

  public double getAngleRadians()
  {
      return ((m_shoulderEncoder.get() - Constants.ArmPositionConstants.shoulderOffset) * Math.PI * 2);
  }

  public double getAbsEncoder()
  {
      return m_shoulderEncoder.get();
  }
}
