// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.AllianceUtils;

//import com.pathplanner.lib.server.PathPlannerServer;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {


 


  public static final ShuffleboardTab driveSettingsTab = Shuffleboard.getTab("Drive Settings");
  public static final ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
  public static final ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve");
  public static final Joystick joystick1 = new Joystick(0);
  
  
  public static final Drivetrain drivetrain = new Drivetrain();

  public static final PoseEstimation poseEstimation = new PoseEstimation();

  public static final SendableChooser<String> drivePresetsChooser = new SendableChooser<>();

  public static Field2d field = new Field2d();
  public static Field2d nodeSelector = new Field2d();

  private final FieldObject2d startingPosition = field.getObject("Starting Position");
  private final FieldObject2d autoBalanceStartingPosition = field.getObject("Auto Balance Starting Position");

  private DriveWithJoysticks driveCommand = new DriveWithJoysticks(drivetrain, poseEstimation, joystick1);
  
 
  public static SendableChooser<String> autoSelector;

  //private static AutoElevator autoElevator;
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {




    if (!DriverStation.isFMSAttached()) {
      
  }

  drivetrain.setDefaultCommand(driveCommand);

  if (autoBalanceStartingPosition.getPoses().isEmpty()) {
    autoBalanceStartingPosition.setPose(AllianceUtils.allianceToField(new Pose2d(new Translation2d(0,0),new Rotation2d())));
  }

    configureBindings();

    autoSelector = new SendableChooser<>();

        autoSelector.setDefaultOption("grid1", "grid1"); // 1 kup en yukari kisa taxi
        //autoSelector.setDefaultOption("grid2", "grid2"); // 1 kup en yukari kisa taxi denge

        //autoSelector.setDefaultOption("grid3", "grid3"); // 1 kup en yukari ortadan denge
        //autoSelector.setDefaultOption("grid4", "grid4"); //1 kup en yukari uzun taxi denge
        //autoSelector.setDefaultOption("grid5", "grid5"); //1 kup en yukari uzun taxi 
  }

  

  

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    

    
    

 





      // Pose Estimation
      
      new JoystickButton(joystick1, 6)
              .onTrue(new InstantCommand(driveCommand::resetFieldOrientation));
      new JoystickButton(joystick1, 7)
              .onTrue(new InstantCommand(() -> poseEstimation.resetPose(
                      new Pose2d(
                              poseEstimation.getEstimatedPose().getTranslation(),
                              new Rotation2d())))); 

      // Driving 
       
      new JoystickButton(joystick1, 1)
              .whileTrue(new RunCommand(
                      drivetrain::setX,
                      drivetrain)); 


   
      
  }

  public Command getAutonomousCommand() {
    Pose2d startingPose = startingPosition.getPose();
    return new SequentialCommandGroup(
    new InstantCommand(() -> poseEstimation.resetPose(startingPose)),

   
    new InstantCommand(() -> poseEstimation.resetPose(startingPose))
    );
  }

  public static Drivetrain getSwerveSubsystem() {
    return drivetrain;
  }





}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

