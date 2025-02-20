// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;


/** An example command that uses an example subsystem. */
public class StagedPitch extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_subsystem;
  private double staged_speed;
  private double curr_pos;
  private double direction;
  private double targ_position;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public StagedPitch(ArmSubsystem subsystem, double speed, double pos) {
    m_subsystem = subsystem;
    staged_speed = speed;
    targ_position = pos;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // curr_pos = m_subsystem.get_pitch_encoder2();
    // if (targ_position > curr_pos){
    //   direction = -1;
      
    // }
    // else if (curr_pos > targ_position){
    //   direction = 1;
    // }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.staged_pitch(staged_speed);
    // curr_pos = m_subsystem.get_pitch_encoder2();
    // if (Math.abs(curr_pos - targ_position) <= 0.01){
    //   m_subsystem.staged_pitch(0);

    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.staged_pitch(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
