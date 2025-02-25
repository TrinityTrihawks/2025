// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


/** An example command that uses an example subsystem. */
public class StagedPitch extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_subsystem;
  private double vel;
  private double curr_pos;
  private double LowLim;
  private double UpLim;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public StagedPitch(ArmSubsystem subsystem, double speed, double pos) {
    m_subsystem = subsystem;
    vel = speed;
    
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
    curr_pos = m_subsystem.get_pitch_encoder2();
    double adjustedVel = vel;
    double slowZoneRange = 1.0;

    if ((vel > 0) && (curr_pos < UpLim)) {
        if (curr_pos >= UpLim - slowZoneRange) {
            double distanceToLimit = UpLim - curr_pos;
            double slowZoneFactor = distanceToLimit / slowZoneRange; // Proportional factor
            adjustedVel = vel * slowZoneFactor;
        }
        m_subsystem.staged_pitch(adjustedVel);
    } else if ((vel < 0) && (curr_pos > LowLim)) {
        if (curr_pos <= LowLim + slowZoneRange) {
            double distanceToLimit = curr_pos - LowLim;
            double slowZoneFactor = distanceToLimit / slowZoneRange; // Proportional factor
            adjustedVel = vel * slowZoneFactor;
        }
        m_subsystem.staged_pitch(adjustedVel);
    } else {
        m_subsystem.staged_pitch(0);
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
}
