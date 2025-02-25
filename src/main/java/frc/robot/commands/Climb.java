package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Claw;

public class Climb extends Command{
    private final ArmSubsystem m_subsystem;
    private final CommandXboxController controller;
  private double vel;
  private double curr_pos;
  private double UpLim = 0;
  private double LowLim = 10;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Climb(ArmSubsystem subsystem, CommandXboxController driverController, double velocity) {
    m_subsystem = subsystem;
    vel=velocity;
    controller = driverController;


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
    curr_pos = m_subsystem.get_climb_encoder2();
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
        m_subsystem.climb(adjustedVel);
    } else if ((vel < 0) && (curr_pos > LowLim)) {
        if (curr_pos <= LowLim + slowZoneRange) {
            double distanceToLimit = curr_pos - LowLim;
            double slowZoneFactor = distanceToLimit / slowZoneRange; // Proportional factor
            adjustedVel = vel * slowZoneFactor;
            SmartDashboard.putString("LIMIT", "SLOW ZONE");
        } else {
            SmartDashboard.putString("LIMIT", "NORMAL ZONE");
        }
        m_subsystem.climb(adjustedVel);
    } else {
        m_subsystem.climb(0);
        controller.setRumble(XboxController.RumbleType.kLeftRumble, 1.0); // Full intensity on left motor
        controller.setRumble(XboxController.RumbleType.kRightRumble, 1.0); // Full intensity on right motor
    }
        SmartDashboard.putString("LIMIT", "STOP");
    }
    
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.climb(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}



