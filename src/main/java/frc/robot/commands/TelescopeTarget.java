package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.TeleArm;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class TelescopeTarget extends Command {
    private final TeleArm m_subsystem;
    private final double target_pos;
    private final double max_output;
    private static final double TOLERANCE = 1;
    private double LowLim = 0;
    private double UpLim = 77;

  
    public TelescopeTarget(TeleArm subsystem, double targetPosition, double maxOutput) {
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
      double curr_pos = m_subsystem.get_tele_encoder1();
      double error = target_pos - curr_pos;
      double output = calculateOutput(error);
      double adjustedVel = output;
  
      if (Math.abs(error) <= TOLERANCE) {
        m_subsystem.telescope(0); // Issue hold command
      } else {
       double slowZoneRange = 1.0;

    if ((output > 0) && (curr_pos < UpLim)) {
        if (curr_pos >= UpLim - slowZoneRange) {
            double distanceToLimit = UpLim - curr_pos;
            double slowZoneFactor = distanceToLimit / slowZoneRange; // Proportional factor
            adjustedVel = output * slowZoneFactor;
            SmartDashboard.putString("LIMIT", "SLOW ZONE");
        } else {
            SmartDashboard.putString("LIMIT", "NORMAL ZONE");
        }
        m_subsystem.telescope(adjustedVel);
    } else if ((output < 0) && (curr_pos > LowLim)) {
        if (curr_pos <= LowLim + slowZoneRange) {
            double distanceToLimit = curr_pos - LowLim;
            double slowZoneFactor = distanceToLimit / slowZoneRange; // Proportional factor
            adjustedVel = output * slowZoneFactor;
            SmartDashboard.putString("LIMIT", "SLOW ZONE");
        } else {
            SmartDashboard.putString("LIMIT", "NORMAL ZONE");
        }
        m_subsystem.telescope(adjustedVel);
    } else {
        m_subsystem.telescope(0);
        SmartDashboard.putString("LIMIT", "STOP");
    }
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
