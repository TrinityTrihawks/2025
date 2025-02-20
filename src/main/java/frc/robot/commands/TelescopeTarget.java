package frc.robot.commands;

import frc.robot.SmartDashboardConstants;
import frc.robot.subsystems.ArmSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class TelescopeTarget extends Command {
    private final ArmSubsystem m_subsystem;
    private final DoubleSupplier target_pos;
    private final double max_output;
    private static final double TOLERANCE = 1;
  
    public TelescopeTarget(ArmSubsystem subsystem, DoubleSupplier targetPosition) {
      m_subsystem = subsystem;
      target_pos = targetPosition;
      max_output = SmartDashboardConstants.TELESCOPE_TARGET_MAX_OUTPUT.getSmartDashboardValue();
      addRequirements(m_subsystem);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      double curr_pos = m_subsystem.get_tele_encoder1();
      double error = target_pos.getAsDouble() - curr_pos;
      double output = calculateOutput(error);
  
      if (Math.abs(error) <= TOLERANCE) {
        m_subsystem.telescope(0); // Issue hold command
      } else {
        m_subsystem.telescope(output);
      }
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      m_subsystem.telescope(0);
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false; // This command should run until the button is released
    }
  
    // Calculate the motor output based on the error
    private double calculateOutput(double error) {
      // Simple proportional control
      double kP = 10.0; // Proportional gain, adjust as needed
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
