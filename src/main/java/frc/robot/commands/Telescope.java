package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.TeleArm;

public class Telescope extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final TeleArm m_subsystem;
  private double vel;
  private double curr_pos;
  private double LowLim = 10;
  private double UpLim = 15;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Telescope(TeleArm subsystem, double velocity) {
    m_subsystem = subsystem;
    vel=velocity;


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
    curr_pos = m_subsystem.get_tele_encoder1();
    double adjustedVel = vel;

    // Define the slow zone range
    double slowZoneRange = 1.0;

    if ((vel > 0) && (curr_pos < UpLim)) {
        if (curr_pos >= UpLim - slowZoneRange) {
            double distanceToLimit = UpLim - curr_pos;
            double slowZoneFactor = distanceToLimit / slowZoneRange; // Proportional factor
            adjustedVel = vel * slowZoneFactor;
            SmartDashboard.putString("LIMIT", "SLOW ZONE");
        } else {
            SmartDashboard.putString("LIMIT", "NORMAL ZONE");
        }
        m_subsystem.telescope(adjustedVel);
    } else if ((vel < 0) && (curr_pos > LowLim)) {
        if (curr_pos <= LowLim + slowZoneRange) {
            double distanceToLimit = curr_pos - LowLim;
            double slowZoneFactor = distanceToLimit / slowZoneRange; // Proportional factor
            adjustedVel = vel * slowZoneFactor;
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

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.telescope(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  private void Plz() {}
}


