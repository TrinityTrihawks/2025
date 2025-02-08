// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class WristTurn extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_subsystem;
  private double vel;
  private double target_pos;
  private double curr_pos;
  private double direction;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public WristTurn(ArmSubsystem subsystem, double velocity) {
    m_subsystem = subsystem;
    vel=velocity;
    //target_pos = position;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  //   if (target_pos > curr_pos){
  //     direction = -1;
      
  //   }
  //   else if (curr_pos > target_pos){
  //     direction = 1;
  //   }
   }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.turn_wrist(vel);
    // curr_pos = m_subsystem.get_wrist_pos2();
    // if (Math.abs(curr_pos - target_pos) <= 0.01){
    //   m_subsystem.staged_pitch(0);

    // }
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
