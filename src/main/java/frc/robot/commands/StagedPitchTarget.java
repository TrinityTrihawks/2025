// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


/** An example command that uses an example subsystem. */
public class StagedPitchTarget extends Command {
  private final ArmSubsystem m_subsystem;
  private final double target_pos;
  private final double max_output;
  private static final double TOLERANCE = 0.005;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public StagedPitchTarget(ArmSubsystem subsystem, double speed, double pos) {
    m_subsystem = subsystem;
    max_output = speed;
    target_pos = pos;
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
    double curr_pos = m_subsystem.get_pitch_encoder2();
      double error = target_pos - curr_pos;
      double output = calculateOutput(error);
  
      if (Math.abs(error) <= TOLERANCE) {
        m_subsystem.staged_pitch(0); // Issue hold command
      } else {
        m_subsystem.staged_pitch(output);
      }

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

  // Calculate the motor output based on the error
  private double calculateOutput(double error) {
    // Simple proportional control
    double kP = 50.0; // Proportional gain, adjust as needed
    double output = kP * error;
    // Limit the output to the maximum output
    if (output > max_output) {
      output = max_output;
    } else if (output < -max_output) {
      output = -max_output;
    }
    return output;
  }
}
