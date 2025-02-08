package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class Telescope extends Command {
    ArmSubsystem m_subsystem;
    double vel;

    public Telescope(ArmSubsystem subsystem, double velocity) {
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
    m_subsystem.telescoping(vel);
    // curr_pos = m_subsystem.get_wrist_pos2();
    // if (Math.abs(curr_pos - target_pos) <= 0.01){
    //   m_subsystem.staged_pitch(0);

    // }
  }
    

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.telescoping(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
