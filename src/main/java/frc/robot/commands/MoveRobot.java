// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;

/** An example command that uses an example subsystem. */
public class MoveRobot extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_subsystem;
  private final CommandXboxController controller;

  private double straight;
  private double strafe;
  private double turn;
  private double Y;
  private double X;
  private double R;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveRobot(DriveSubsystem subsystem, CommandXboxController driverController) {
    m_subsystem = subsystem;
    controller = driverController;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double threshold = 0.1;

    straight = controller.getLeftY();
    strafe = controller.getRightX();
    turn = controller.getLeftX();

    Y = Math.pow(straight, 3) * OperatorConstants.straightmax;
    X = Math.pow(strafe, 3) * OperatorConstants.rotationemax;
    R = Math.pow(turn, 3) * OperatorConstants.turnmax;

    if(Y < threshold && Y > -threshold){
      Y = 0;
    }
    if(X < threshold && X > -threshold){
      X = 0;
    }
    if(R < threshold && R > -threshold){
      R = 0;
    }

    m_subsystem.drive(-Y, R, X);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
