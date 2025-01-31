// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveModulePIDValues;
import frc.robot.Constants.SteerModulePIDValues;
import frc.robot.Constants.SwerveDriveModuleConstants;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

  /** Creates a new SwerveModule. */
  public class SwerveModule
  { 
    
    private TalonFX m_driveMotor;
    private TalonFX m_angleMotor;
    private Rotation2d m_offset;
    private CANCoder m_moduleSteeringEncoder;

      /**
     * Constructs a SwerveModule.
     *
     * @param driveMotorChannel ID for the drive motor.
     * @param steeringMotorChannel ID for the turning motor.
     */
    public SwerveModule(TalonFX driveMotor, TalonFX angleMotor, CANCoder canCoder, Rotation2d offset)
    {
        m_driveMotor = driveMotor;
        m_angleMotor = angleMotor;
        m_moduleSteeringEncoder = canCoder;
        m_offset = offset;

        CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
        canCoderConfiguration.magnetOffsetDegrees = m_offset.getDegrees();
        canCoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        canCoderConfiguration.sensorDirection = false;
        canCoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        canCoder.configAllSettings(canCoderConfiguration);
        canCoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 5);

        m_angleMotor.configFactoryDefault();
        m_angleMotor.setNeutralMode(NeutralMode.Brake);

        TalonFXConfiguration angleTalonConfig = new TalonFXConfiguration();
    
        angleTalonConfig.slot0.allowableClosedloopError = SteerModulePIDValues.k_ToleranceInDegrees;
        angleTalonConfig.remoteFilter0.remoteSensorDeviceID = m_moduleSteeringEncoder.getDeviceID();
        angleTalonConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
        angleTalonConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;

        angleTalonConfig.supplyCurrLimit.enable = true;
        angleTalonConfig.supplyCurrLimit.triggerThresholdCurrent = 45;
        angleTalonConfig.supplyCurrLimit.currentLimit = 35;

        angleTalonConfig.slot0.kP = SteerModulePIDValues.k_steerP;
        angleTalonConfig.slot0.kI = SteerModulePIDValues.k_steerI;
        angleTalonConfig.slot0.kD = SteerModulePIDValues.k_steerD;

        m_angleMotor.configAllSettings(angleTalonConfig);
        m_angleMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
        m_angleMotor.setSensorPhase(true);
        m_angleMotor.setInverted(true);

        m_driveMotor.configFactoryDefault();
        m_driveMotor.setNeutralMode(NeutralMode.Brake);

        TalonFXConfiguration driveTalonConfig = new TalonFXConfiguration();
    
        driveTalonConfig.slot0.allowableClosedloopError = DriveModulePIDValues.k_ToleranceInTicks;
        driveTalonConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        
        driveTalonConfig.slot0.kP = DriveModulePIDValues.k_driveP;
        driveTalonConfig.slot0.kI = DriveModulePIDValues.k_driveI;
        driveTalonConfig.slot0.kD = DriveModulePIDValues.k_driveD;
        driveTalonConfig.slot0.kF = DriveModulePIDValues.k_driveFF;

        driveTalonConfig.supplyCurrLimit.enable = true;
        driveTalonConfig.supplyCurrLimit.triggerThresholdCurrent = 45;
        driveTalonConfig.supplyCurrLimit.currentLimit = 35;
        m_driveMotor.configAllSettings(driveTalonConfig);
        m_driveMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
        m_driveMotor.setInverted(true);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean optimize)
    {      
       //Steering Motor Calc
        SwerveModuleState state;
        Rotation2d currentRotation = getAngle();
        if(optimize)
        {
           state = SwerveModuleState.optimize(desiredState, currentRotation);
        } 
        else 
        {
            state = desiredState;
        }
        Rotation2d rotationDelta = state.angle.minus(currentRotation); //takes our current rotatation and subtracts the last state rotation
       
        double deltaTicks = calculateDeltaTicks(rotationDelta);
        double currentTicks = calculateCurrentTicks();
        double desiredTicks = currentTicks + deltaTicks;

        if(Math.abs(desiredState.speedMetersPerSecond)  <= SwerveDriveModuleConstants.k_MaxTeleSpeed * 0.05)
        {
           desiredTicks = currentTicks;
        }
        else
        {
            desiredTicks = currentTicks + deltaTicks;
        }

        m_angleMotor.set(TalonFXControlMode.Position, filterAngleMotorDeadband(desiredTicks));
        
        m_driveMotor.set(TalonFXControlMode.PercentOutput, state.speedMetersPerSecond / SwerveDriveModuleConstants.k_MaxTeleSpeed);
    }

    public void setDesiredAutoState(SwerveModuleState desiredState, boolean optimize)
    {      
       //Steering Motor Calc
        SwerveModuleState state;
        Rotation2d currentRotation = getAngle();
        if(optimize)
        {
           state = SwerveModuleState.optimize(desiredState, currentRotation);
        } 
        else 
        {
            state = desiredState;
        }
        Rotation2d rotationDelta = state.angle.minus(currentRotation); //takes our current rotatation and subtracts the last state rotation
       
        double deltaTicks = calculateDeltaTicks(rotationDelta);
        double currentTicks = calculateCurrentTicks();
        double desiredTicks = currentTicks + deltaTicks;

        desiredTicks = currentTicks + deltaTicks;

        m_angleMotor.set(TalonFXControlMode.Position, desiredTicks);
        
        m_driveMotor.set(TalonFXControlMode.Velocity, state.speedMetersPerSecond / SwerveDriveModuleConstants.k_driveDistancePerPulse);
    }

    public void setZeroState()
    {
        SwerveModuleState state = new SwerveModuleState(0, new Rotation2d(0));
        Rotation2d currentRotation = getAngle();
        Rotation2d rotationDelta = state.angle.minus(currentRotation);
        double deltaTicks = calculateDeltaTicks(rotationDelta);
        double currentTicks = calculateCurrentTicks();
        double desiredTicks = currentTicks + deltaTicks;
        
        m_angleMotor.set(TalonFXControlMode.Position, desiredTicks);   
    }

    public void setLockState(double rotation)
    {
        SwerveModuleState state = new SwerveModuleState(0, new Rotation2d(rotation));
        Rotation2d currentRotation = getAngle();
        Rotation2d rotationDelta = state.angle.minus(currentRotation);
        double deltaTicks = calculateDeltaTicks(rotationDelta);
        double currentTicks = calculateCurrentTicks();
        double desiredTicks = currentTicks + deltaTicks;
        
        m_angleMotor.set(TalonFXControlMode.Position, desiredTicks);   
    }

    public Rotation2d getAngle()
    {
        return Rotation2d.fromDegrees(m_moduleSteeringEncoder.getAbsolutePosition());
    }

    public double calculateDeltaTicks(Rotation2d rotationDelta) 
    {
        return (rotationDelta.getDegrees() / 360) * SwerveDriveModuleConstants.k_driveEncoderTicksPerRotation;
    }

    public double calculateCurrentTicks() {
        double currentTicks = m_moduleSteeringEncoder.getPosition() / SwerveDriveModuleConstants.k_steerFeedbackCoefficient; /*m_moduleSteeringEncoder.configGetFeedbackCoefficient()*/;
        return currentTicks;
    }

    public double filterAngleMotorDeadband(double setAngle)
    {
        if(Math.abs(setAngle) > SteerModulePIDValues.k_steerDeadband)
        {
           return setAngle;
        } 
        return 0.0;
    }

    public void resetDriveEncoder()
    {
        m_driveMotor.setSelectedSensorPosition(0);
        m_moduleSteeringEncoder.setPosition(0);  
    }

    public SwerveModuleState getState()
    {
        double driveSpeed = (m_driveMotor.getSelectedSensorVelocity() * (10.0 / 2048) * Math.PI * SwerveDriveModuleConstants.k_wheelDiameter) / SwerveDriveModuleConstants.k_gearRatio;
        return new SwerveModuleState(driveSpeed , Rotation2d.fromDegrees(m_moduleSteeringEncoder.getPosition()));
    }

    public double getDriveVelocity()
    {
        return (m_driveMotor.getSelectedSensorVelocity() * (10.0 / 2048) * Math.PI * SwerveDriveModuleConstants.k_wheelDiameter) / SwerveDriveModuleConstants.k_gearRatio;
    }

    public double getRotationDegrees()
    {
        return m_moduleSteeringEncoder.getPosition();
    }

    public double getDriveEncoder() {
        return m_driveMotor.getSelectedSensorPosition();
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(m_driveMotor.getSelectedSensorPosition() * Math.PI * SwerveDriveModuleConstants.k_wheelDiameter / (SwerveDriveModuleConstants.k_gearRatio * 2048), new Rotation2d(m_moduleSteeringEncoder.getPosition() * Math.PI / 180));
    }
    
}
