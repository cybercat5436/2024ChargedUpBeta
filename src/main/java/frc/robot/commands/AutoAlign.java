// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoAlign extends Command {
  private LimeLight limeLight;
  private SwerveSubsystem swerveSubsystem;
  private boolean inSight = false;
  private double tx;
  private double ty;
  private double tz;
  private double targetTx = 0.0;
  private double targetTz = -0.8;
  private double txError, tzError, ySpeed, xSpeed;
  private double[] botpose;
  private double kXSpeed = 0.4;
  private double kZSpeed = 0.3;

  
  /** Creates a new AutoAlign. */
  public AutoAlign(LimeLight limeLight, SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limeLight = limeLight;
    this.swerveSubsystem = swerveSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(!limeLight.getVisionTargetStatus()){
      System.out.println("AprilTag is not in sight.");
    }else{
      inSight = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(limeLight.getVisionTargetStatus()){
      // for(double x:limeLight.getBotPose()){
      //   System.out.print(x+" ");
      // }
      // System.out.println();
      botpose = limeLight.getBotPose();
      tx = botpose[0];
      ty = botpose[1];
      tz = botpose[2];
      
      txError = tx - targetTx;
      tzError = tz - targetTz;
      System.out.println("txError: "+txError);
      System.out.println("tzError: "+tzError);

      xSpeed = kZSpeed * tzError;
      ySpeed = kXSpeed * txError * -1;
      
      SmartDashboard.putNumber("AutoAlign xSpeed", xSpeed);
      SmartDashboard.putNumber("AutoAlign ySpeed", ySpeed);
      ChassisSpeeds chassisSpeeds;
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, 0);
      SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
      swerveSubsystem.setModuleStates(moduleStates);

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(tzError)<0.2 && Math.abs(txError)<0.2){
      return true;
    }
    return false;
  }
}
