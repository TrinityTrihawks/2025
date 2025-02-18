package frc.robot.commands;

import frc.robot.subsystems.Claw;
import edu.wpi.first.wpilibj2.command.Command;

public class WristTurnTarget extends Command {
    private final Claw m_subsystem;
    private final double target_pos;
    private final double max_output;
    private static final double TOLERANCE = 0.005;
  
    public WristTurnTarget(Claw subsystem, double targetPosition, double maxOutput) {
      m_subsystem = subsystem;
      target_pos = targetPosition;
      max_output = maxOutput;
      addRequirements(m_subsystem);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      double curr_pos = m_subsystem.get_wrist_encoder2();
      double error = target_pos - curr_pos;
      double output = calculateOutput(error);
  
      if (Math.abs(error) <= TOLERANCE) {
        m_subsystem.turn_wrist(0); // Issue hold command
      } else {
        m_subsystem.turn_wrist(output);
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
      return false; // This command should run until the button is released
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