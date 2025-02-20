// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.Autos;
import frc.robot.commands.Climb;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.GripperIntake;
import frc.robot.commands.MoveRobot;
import frc.robot.commands.StagedPitch;
import frc.robot.commands.Telescope;
import frc.robot.commands.TelescopeTarget;
import frc.robot.commands.WristTurn;
import frc.robot.commands.WristTurnTarget;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ArmSubsystem m_robotArm = new ArmSubsystem();
  private final Claw m_claw = new Claw();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_SubdriverController = new CommandXboxController(0);
  private final CommandXboxController m_driverController = new CommandXboxController(1);
  private double intake_speed;
  private double output_speed;
  private double wrist_speed;
  private double pitch_speed;
  private double telescope_speed;
  private double climb_speed;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger00 bindings
    configureBindings();
    System.out.print("something");

    m_robotDrive.setDefaultCommand(
        new MoveRobot(m_robotDrive, m_driverController));
    for (SmartDashboardConstants robotDefault: SmartDashboardConstants.values()) {
        robotDefault.setSmartDashboardDefault();
    }
    SmartDashboard.putNumber("Pitch Speed", 1);
    SmartDashboard.putNumber("Wrist Speed", 1);
    SmartDashboard.putNumber("Intake Speed", 1);
    SmartDashboard.putNumber("Output Speed", 1);
    SmartDashboard.putNumber("Climb Speed", 1);
    SmartDashboard.putNumber("Stage Speed", 1);
  }

  /* This moveRobot() function was moved to the MoveRobot.java command script. */

  void moveRobot() {
    double threshold = 0.1;
    double leftY = m_driverController.getLeftY() * OperatorConstants.straightmax;
    double rightX = m_driverController.getRightX() * OperatorConstants.strafemax;
    double leftX = m_driverController.getLeftX() * OperatorConstants.turnmax;

    if(leftY < threshold && leftY > -threshold){
      leftY = 0;
    }
    if(leftX < threshold && leftX > -threshold){
      leftX = 0;
    }
    if(rightX < threshold && rightX > -threshold){
      rightX = 0;
    }

    m_robotDrive.drive(
      -leftY,
      rightX,
      leftX
    );
  }
  public void SmartDashboardValues() {
    pitch_speed = SmartDashboard.getNumber("Pitch Speed", 1);
    wrist_speed = SmartDashboard.getNumber("Wrist Speed", 1);
    intake_speed = SmartDashboard.getNumber("Intake Speed", 1);
    output_speed = SmartDashboard.getNumber("Output Speed", 1);
    climb_speed = SmartDashboard.getNumber("Climb Speed", 1);
    telescope_speed = SmartDashboard.getNumber("Stage Speed", 1);
    SmartDashboard.putNumber("Climb Motor Encoder", m_robotArm.get_climb_encoder1());
    SmartDashboard.putNumber("Climb Alt Encoder", m_robotArm.get_climb_encoder2());
    SmartDashboard.putNumber("Pitch Motor Encoder", m_robotArm.get_pitch_encoder1());
    SmartDashboard.putNumber("Pitch Alt Encoder", m_robotArm.get_pitch_encoder2());
    SmartDashboard.putNumber("Wrist Motor Encoder", m_claw.get_wrist_encoder1());
    SmartDashboard.putNumber("Wrist Alt Encoder", m_claw.get_wrist_encoder2());
    SmartDashboard.putNumber("Tele Motor Encoder", m_robotArm.get_tele_encoder1());

  }


  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
     m_SubdriverController.a().whileTrue(new StagedPitch(m_robotArm,12,0));
     m_SubdriverController.b().whileTrue(new StagedPitch(m_robotArm,-12,0.88));
    //  m_SubdriverController.x().whileTrue(new StagedPitch(m_robotArm,pitch_speed,0.63));

     m_SubdriverController.leftBumper().whileTrue(new WristTurnTarget(m_claw,0.5, 1));
     m_SubdriverController.rightBumper().whileTrue(new WristTurnTarget(m_claw, 0, 1));

     m_SubdriverController.rightTrigger().whileTrue(new GripperIntake(m_claw, false));
     m_SubdriverController.leftTrigger().whileTrue(new GripperIntake(m_claw, true));

    //  m_driverController.x().whileTrue(new Climb(m_robotArm,6));
    //  m_driverController.y().whileTrue(new Climb(m_robotArm,-1 * 6));

     m_SubdriverController.x().whileTrue(new Telescope(m_robotArm, false));
     m_SubdriverController.y().whileTrue(new Telescope(m_robotArm, true));

     m_SubdriverController.povDown().whileTrue(new TelescopeTarget(m_robotArm, () -> SmartDashboardConstants.TELESCOPE_TARGET_POSITION_L1.getSmartDashboardValue())); // L1 scoring
     m_SubdriverController.povLeft().whileTrue(new TelescopeTarget(m_robotArm, () -> SmartDashboardConstants.TELESCOPE_TARGET_POSITION_L2.getSmartDashboardValue())); // L2 scoring
     m_SubdriverController.povUp().whileTrue(new TelescopeTarget(m_robotArm, () -> SmartDashboardConstants.TELESCOPE_TARGET_POSITION_L3.getSmartDashboardValue())); // L3 scoring
     m_SubdriverController.povRight().whileTrue(new TelescopeTarget(m_robotArm, () -> SmartDashboardConstants.TELESCOPE_TARGET_POSITION_RETRACT.getSmartDashboardValue())); // retract arm
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  Command autoCommand () {
    return new AutoDrive(m_robotDrive, 2);
  }
  
}
