package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Claw;

public class Telescope extends Command{
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
  public Telescope(ArmSubsystem subsystem, double velocity) {
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
    if ((vel > 0 && curr_pos < UpLim) || (vel < 0 && curr_pos > LowLim)) {
      m_subsystem.telescope(vel);
    } else {
      m_subsystem.telescope(0);
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


