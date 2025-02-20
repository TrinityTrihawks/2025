package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SmartDashboardConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Claw;

public class Telescope extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_subsystem;
  private double vel;
  private boolean reverse;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Telescope(ArmSubsystem subsystem, boolean reverse) {
    m_subsystem = subsystem;
    this.vel=SmartDashboardConstants.TELESCOPE_VELOCITY.getSmartDashboardValue();
    this.reverse = reverse;

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
    double velocity = (reverse ? -1 : 1) * SmartDashboardConstants.TELESCOPE_VELOCITY.getSmartDashboardValue();
    m_subsystem.telescope(velocity);
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
}


