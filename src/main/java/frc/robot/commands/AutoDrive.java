package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class AutoDrive extends Command{ 
    private final DriveSubsystem driveSubsystem;
    private final double duration;
    private final Timer timer;

    public AutoDrive(DriveSubsystem driveSubsystem, double duration) {
        this.driveSubsystem = driveSubsystem;
        this.duration = duration;
        this.timer = new Timer();
        addRequirements(driveSubsystem);
    }
    @Override
    public void initialize () {
        timer.reset();
        timer.start();
    }
    @Override
    public void execute() {
        driveSubsystem.drive(-0.2,0,0);
    }
    @Override
    public boolean isFinished() {
        return timer.get() >= duration;
    }
    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0,0,0);
    }

}
