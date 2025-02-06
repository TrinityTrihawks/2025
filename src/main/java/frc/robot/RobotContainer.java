// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.GripperIntake;
import frc.robot.commands.MoveRobot;
import frc.robot.commands.StagedPitch;
import frc.robot.commands.WristTurn;
import frc.robot.subsystems.ArmSubsystem;
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

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger00 bindings
    configureBindings();
    System.out.print("something");

    m_robotDrive.setDefaultCommand(
        new MoveRobot(m_robotDrive, m_driverController));
  }

  /* This moveRobot() function was moved to the MoveRobot.java command script. */

  // void moveRobot() {
  //   double threshold = 0.1;
  //   double leftY = m_driverController.getLeftY() * OperatorConstants.straightmax;
  //   double rightX = m_driverController.getRightX() * OperatorConstants.strafemax;
  //   double leftX = m_driverController.getLeftX() * OperatorConstants.turnmax;

  //   if(leftY < threshold && leftY > -threshold){
  //     leftY = 0;
  //   }
  //   if(leftX < threshold && leftX > -threshold){
  //     leftX = 0;
  //   }
  //   if(rightX < threshold && rightX > -threshold){
  //     rightX = 0;
  //   }

  //   m_robotDrive.drive(
  //     -leftY,
  //     rightX,
  //     leftX
  //   );
  // }

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
     m_driverController.a().whileTrue(new StagedPitch(m_robotArm,1,0));
     m_driverController.b().toggleOnTrue(new StagedPitch(m_robotArm,1,0.88));
     m_driverController.x().toggleOnTrue(new StagedPitch(m_robotArm,1,0.63));
     m_driverController.leftBumper().whileTrue(new WristTurn(m_robotArm,1, 0));
     m_driverController.rightBumper().whileTrue(new WristTurn(m_robotArm,1, 0.75));
     m_driverController.rightTrigger().whileTrue(new GripperIntake(m_robotArm,1));
     m_driverController.leftTrigger().whileTrue(new GripperIntake(m_robotArm,-1));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}
