// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Claw;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class WristTurn extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Claw m_subsystem;
  private double vel;
  private double target_pos;
  private double curr_pos;
  private double speed_ratio;
  private double direction;
  private double error;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public WristTurn(Claw subsystem, double output, double position) {
    m_subsystem = subsystem;
    vel=output;
    target_pos = position;
    

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    curr_pos = m_subsystem.get_wrist_encoder2();
    speed_ratio = 1; // This is the ratio of which defines the speed so change to aclimate the encoder values
    error = curr_pos - target_pos;
    vel = speed_ratio * error;
    m_subsystem.turn_wrist(vel);
    if (Math.abs(error) <= 0.01){
      m_subsystem.turn_wrist(0);

    }
  }
    

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.turn_wrist(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
