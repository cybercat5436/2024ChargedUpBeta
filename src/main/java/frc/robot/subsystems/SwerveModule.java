// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.enums.WheelPosition;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class SwerveModule implements Sendable{

  private final CANSparkMax driveMotor;
  private final CANSparkMax turningMotor;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turningEncoder;

  private final PIDController turningPidController;
  private final SparkMaxPIDController velocityPidController;

  private final AnalogInput absoluteEncoder;
  private final boolean absoluteEncoderReversed;
  private final double absoluteEncoderOffsetRad;

  public final WheelPosition wheelPosition;

  public SwerveModuleState desiredState = new SwerveModuleState();
  
  public double kP = 0.45;
  public double kFF = 0.225;

  /** Creates a new SwerveModule. */
  public SwerveModule(WheelPosition wheelPosition, int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
   int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed, IdleMode driveMode, IdleMode turningMode) {

    this.wheelPosition = wheelPosition;
    this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
    this.absoluteEncoderReversed = absoluteEncoderReversed;
    absoluteEncoder = new AnalogInput(absoluteEncoderId);

    driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
    turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

    driveMotor.clearFaults();
    turningMotor.clearFaults();

    driveMotor.restoreFactoryDefaults();
    turningMotor.restoreFactoryDefaults();

    driveMotor.setSmartCurrentLimit(50, 50);
    turningMotor.setSmartCurrentLimit(30, 30);

    driveMotor.setInverted(driveMotorReversed);
    turningMotor.setInverted(turningMotorReversed);

    driveMotor.setIdleMode(driveMode);
    turningMotor.setIdleMode(turningMode);

    driveEncoder = driveMotor.getEncoder();
    turningEncoder = turningMotor.getEncoder();

    driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
    driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
    turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
    turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

    turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
    turningPidController.enableContinuousInput(-Math.PI, Math.PI);

    velocityPidController = driveMotor.getPIDController();
    velocityPidController.setP(kP);
    velocityPidController.setFF(kFF); 

    resetDriveEncoders();
    resetTurningEncoderWithAbsolute();
  }

  /*
  public double getDrivePosition() {
    return driverEncoder.getPosition();
  }

  public double getSpinPosition() {
    return spinEncoder.getPosition();
  }
  
  public double getDriveVelocity() {
    return driverEncoder.getVelocity();
  }

  public double getSpinVelocity() {
    return spinEncoder.getVelocity();
  }

  public void resetEncoders() {
    driverEncoder.setPosition(0);
    spinEncoder.setPosition(0);
  }
  */

  public void resetDriveEncoders() {
    driveEncoder.setPosition(0.0);
    
    // DataLogManager.log(String.format("About to reset encoders for position %s", this.wheelPosition.name()));
    // DataLogManager.log(String.format("turning Encoder %.2f", turningEncoder.getPosition()));
    // DataLogManager.log(String.format("absoluteEncoder %.2f", this.getAbsoluteEncoderRadians()));
    // turningEncoder.setPosition(getAbsoluteEncoderRadians());
    // DataLogManager.log(String.format("After reset encoders for position %s", this.wheelPosition.name()));
    // DataLogManager.log(String.format("after zero turningEncoder %.2f", turningEncoder.getPosition()));
    // DataLogManager.log(String.format("after zero absoluteEncoder %.2f", this.getAbsoluteEncoderRadians()));
  }
  public void resetTurningEncoderWithAbsolute () {
    turningEncoder.setPosition(getAbsoluteEncoderRadians());

  }
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition( driveEncoder.getPosition(), new Rotation2d(turningEncoder.getPosition()));
  }
  public void zeroTurningEncoder(){
    turningEncoder.setPosition(0);  
  }

  public double getDrivePosition(){
    return driveEncoder.getPosition();
  }

  public double getTurningPosition(){
    return turningEncoder.getPosition();
  }

  public double getDriveVelocity(){
    return driveEncoder.getVelocity();
  }

  public AnalogInput getAbsoluteEncoder(){
    return this.absoluteEncoder;
  }



  public SwerveModuleState getActualState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }

  public SwerveModuleState getDesiredState(){
      return new SwerveModuleState(this.desiredState.speedMetersPerSecond, this.desiredState.angle);

  }

  //definetly ot right but gets rid of error
  // private SwerveModuleState SwerveModuleState(double driveVelocity, Rotation2d rotation2d) {
  //   return null;
  // }

  /**
   * Applies the provided state to the individual module
   * @param state
   */
  public void setDesiredState(SwerveModuleState state){
    // set the desired state prior to processing
    this.desiredState = state;

    // set both motors to zero if below speed threshold
    if (Math.abs(state.speedMetersPerSecond) < 0.01) {
      this.desiredState = new SwerveModuleState(0.0, state.angle);
      this.stop();
      return;
    }

    state = SwerveModuleState.optimize(state, getActualState().angle);

    // set drive power using linear correlation with max speed
    // double driveMotorPower = state.speedMetersPerSecond /DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
    // driveMotor.set(driveMotorPower);
    
    // set drive power using velocity PID controller control
    velocityPidController.setReference(state.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);

    turningMotor.set(turningPidController.calculate(turningEncoder.getPosition(), state.angle.getRadians()));

    // SmartDashboard.putString(String.format("%s Modue State", this.wheelPosition.name()), state.toString());
    
    // SmartDashboard.putNumber(String.format("%s Drive Motor Power", this.wheelPosition.name()) , driveMotorPower);

  }


  public double getAbsoluteEncoderRadians() {
    // double angle = absoluteEncoder.getVoltage() / RobotController.getCurrent5V();
    double angle = absoluteEncoder.getVoltage() / 5.0;
    angle = (Math.PI * 2) * angle - Math.PI;
    if (absoluteEncoderReversed){
      angle *= -1;
    } 
    angle -= absoluteEncoderOffsetRad;
    angle = boundAngle(angle);
    return angle;
  }

  private double boundAngle(double inputAngleRad){
    double outputAngleRad = inputAngleRad;
    if (outputAngleRad <= -Math.PI){
      outputAngleRad += (2*Math.PI);
    } else if(outputAngleRad > (Math.PI)){
      outputAngleRad -= (2 * Math.PI);
    }
    return outputAngleRad;

  }



  public void stop() {
    driveMotor.set(0);
    turningMotor.set(0);
  }

  public String getStateStringNoLabel(SwerveModuleState state){
    return String.format("%6.2f m/s @ %6.1f deg", state.speedMetersPerSecond, state.angle.getDegrees());

  }

  @Override
  public void initSendable(SendableBuilder builder) {
    // TODO Auto-generated method stub
    builder.addDoubleProperty("Power From Module", () -> this.getDriveVelocity(), null);
    // builder.addDoubleProperty("Physical Restraints", () -> DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond, null);
  }

  @Override
  public String toString(){
    String label = this.wheelPosition.name();
    return String.format("%s : %s", label, this.getStateStringNoLabel(this.getActualState()));
  }

  public String toStringDesiredState(){
    String label = this.wheelPosition.name();
    return String.format("%s : %s", label, this.getStateStringNoLabel(this.getDesiredState()));
  }




  /**@Override
  public void periodic() {
   This method will be called once per scheduler run
  } **/
}
