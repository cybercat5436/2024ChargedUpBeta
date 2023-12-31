// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AutonIntakeCommand;
import frc.robot.commands.AutonomousAutoBalance;
import frc.robot.commands.ArmGoToHighMotionMagic;
import frc.robot.commands.AbsoluteEncoderCalibration;
import frc.robot.commands.ArmGoToMid;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.AutonArmDownCommand;
import frc.robot.commands.ExtenderRetractToZero;
import frc.robot.commands.ManualEncoderCalibration;
import frc.robot.commands.MoveToFulcrum;
import frc.robot.commands.SetTo90;
import frc.robot.commands.OrientCone;
import frc.robot.commands.SeekFulcrum;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Orienter;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    public boolean halfSpeed = false;
    private final LimeLight limeLightGrid = new LimeLight("limelight");
    private final LimeLight limeLightOrient = new LimeLight("limelight-orient");
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final Orienter orienter = new Orienter(limeLightOrient);
    private final Claw claw = new Claw();
    private final Arm arm = new Arm();
    private final Intake intake = new Intake();
    private final Extender extender = new Extender();
    private final CommandXboxController primaryController = new CommandXboxController(1);
    private final CommandXboxController secondaryController = new CommandXboxController(0);
    // private final SendableChooser<Command> autonChooser = new SendableChooser<>();

    private final SendableChooser<Command> autonChooser;

  
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
        swerveSubsystem,
        () -> -primaryController.getLeftY(),
        () -> -primaryController.getLeftX(),
        () -> -primaryController.getRightX(),
        () -> !primaryController.start().getAsBoolean(),
        // () -> primaryController.leftBumper().getAsBoolean(),
        () -> primaryController.rightTrigger().getAsBoolean(),
        () -> primaryController.y().getAsBoolean(),
        //() -> primaryController.rightBumper().getAsBoolean(),
        () -> primaryController.x().getAsBoolean(),
        () -> primaryController.getLeftTriggerAxis(),
        () -> primaryController.getRightTriggerAxis(),
        limeLightGrid));

      // Configure the button bindings
      ManualEncoderCalibration manualEncoderCalibration = new ManualEncoderCalibration(swerveSubsystem);  
      AbsoluteEncoderCalibration absoluteEncoderCalibration = new AbsoluteEncoderCalibration(swerveSubsystem);           
      primaryController.b()

          .onTrue(new OrientCone(orienter, limeLightGrid));

      SmartDashboard.putData(manualEncoderCalibration);
      SmartDashboard.putData(absoluteEncoderCalibration);
      configureButtonBindings();

      Utils util = new Utils();

      SmartDashboard.putData(new InstantCommand(() -> swerveSubsystem.zeroIntegrator()));

      SequentialCommandGroup stateMachineAutoBalance = new SequentialCommandGroup(
        new SeekFulcrum(swerveSubsystem),
        new MoveToFulcrum(swerveSubsystem));
      SmartDashboard.putData(stateMachineAutoBalance);

      // Register Named Commands for Path Planner
      registerNamedCommands();
      // Build an auto chooser. This will use Commands.none() as the default option.
      String defaultAuton = AutoBuilder.getAllAutoNames().isEmpty() ? "" : AutoBuilder.getAllAutoNames().get(0);
      autonChooser = AutoBuilder.buildAutoChooser(defaultAuton);
      SmartDashboard.putData("Auton Chooser", autonChooser);
}
  
    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

      // enter semi-autonomous state for grabbing cone when hold start
      primaryController.start().whileTrue(new AutoAlign(limeLightGrid, swerveSubsystem));

      //Arm Buttons
      secondaryController.pov(0).whileTrue(new InstantCommand(()->arm.armUp()));
      secondaryController.pov(-1).whileTrue(new InstantCommand(()->arm.stopArm()));
      secondaryController.pov(180).whileTrue(new InstantCommand(()->arm.armDown()));

      //Extender Buttons
      secondaryController.b().onTrue(new InstantCommand(()->extender.extend()))
        .onFalse(new InstantCommand(()->extender.stopExtend()));  
      //secondaryController.b().and(() -> secondaryController.leftTrigger().getAsBoolean()).onTrue();    
      secondaryController.x().onTrue(new InstantCommand(()->extender.retract()))
        .onFalse(new InstantCommand(()->extender.stopExtend()));

      secondaryController.pov(90).onTrue(new InstantCommand(()->orienter.microwaveManualSpin()))
        .onFalse(new InstantCommand(()->orienter.stopMicrowave()));
      secondaryController.pov(270).onTrue(new InstantCommand(()->orienter.microwaveReverseManualSpin()))
        .onFalse(new InstantCommand(()->orienter.stopMicrowave()));
      

      //Claw Buttons
      secondaryController.rightBumper().onTrue(new InstantCommand(()->claw.clawRelease()))
        .onFalse(new InstantCommand(()->claw.stopGrab()));
      secondaryController.leftBumper().onTrue(new InstantCommand(()->claw.clawGrab()))
        .onFalse(new InstantCommand(()->claw.stopGrab()));
     
      
      
      //Intake Buttons
      primaryController.leftBumper().onTrue(new InstantCommand(()->intake.intakeFeedIn()))
        .onFalse(new InstantCommand(()->intake.stopIntake()));
      primaryController.rightBumper().onTrue(new InstantCommand(()->intake.intakeFeedOut()))
        .onFalse(new InstantCommand(()->intake.stopIntake()));
      

      //Auto command groups
      secondaryController.start().onTrue(new SequentialCommandGroup(
          new ArmGoToHighMotionMagic(arm),
          new InstantCommand(()->extender.extendHighGoal()) 
      ));
      secondaryController.back().onTrue(new SequentialCommandGroup(
          new ArmGoToMid(arm),
          new InstantCommand(()->extender.extendMidGoal())
      ));
      secondaryController.rightStick().onTrue(new SequentialCommandGroup(
          new ExtenderRetractToZero(extender),
          new InstantCommand(()->arm.armMoveToZeroPosition()) 
      ));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
      return autonChooser.getSelected();
              
  }

  private void registerNamedCommands(){
    Utils util = new Utils();
    NamedCommands.registerCommand("scoreHighGoal", util.scoreHighGoal(extender, claw, arm));
    NamedCommands.registerCommand("foldArm", util.retractArm(extender, claw, arm));
    NamedCommands.registerCommand("startIntake", new InstantCommand(() -> intake.intakeFeedIn()));
    NamedCommands.registerCommand("stopIntake", new InstantCommand(() -> intake.stopIntake()));
    NamedCommands.registerCommand("score", 
      new InstantCommand(() -> intake.intakeFeedOut()).repeatedly().withTimeout(3.0)
        .andThen(new InstantCommand(() -> intake.stopIntake()))
    );
  }

  /**
   * Looks at the auton command selected in auton chooser.  If no starting pose is present, it sets the robot's starting pose
   * to the initial pose of the first path
   * @param autonName:  name of auton path selected
   */
  public void setStartingPoseIfMissing(String autonName){
    System.out.println("Selected Auton: " + autonName);
    String poseString = "Unknown";
    
    try {
      poseString = PathPlannerAuto.getStaringPoseFromAutoFile(autonName).toString();
      System.out.println("StartingPose from Auto: " + poseString);

    } catch (RuntimeException e){
      // Exception thrown if starting pose is null in PathPlanner Auton file
      System.out.println("No starting pose detected in Auton file!");
      
      // get a list of all paths present in the auton file
      List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile(autonName);
      // make sure the list isn't empty
      if(!paths.isEmpty()){
        PathPlannerPath firstPath = paths.get(0);
        Pose2d initialPose = firstPath.getPreviewStartingHolonomicPose();
        System.out.println("Starting pose from first path: " + initialPose);
        System.out.println("Setting robot pose...");
        swerveSubsystem.resetOdometry(initialPose);
        poseString = initialPose.toString();
      } else{
        // no pose provided and no paths present
        System.out.println("Initial Pose is unknown, potential issue!!!");
        // throw new RuntimeException("No Initial Pose for the robot provided");
      }
    }

    System.out.println("Exiting setStartingPoseIfMissing with robot pose: " + poseString);
  }

}
  