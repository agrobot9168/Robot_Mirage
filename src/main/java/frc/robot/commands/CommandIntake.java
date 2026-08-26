package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;

public class CommandIntake extends Command{
    private final IntakeSubsystem intake = RobotContainer.intakeSubsystem;
    private double pose;
    private double vel;

    public CommandIntake(double pose, double vel) {
        addRequirements(intake);
        this.pose = pose;
        this.vel = vel;

    }
    @Override
    public void initialize() {
        // intake.SetPose_Inclina(pose);
        intake.SetVel_Colet(vel);
    }
    @Override
    public void execute() {
        SmartDashboard.putNumber("GETPosition_", intake.GetPose_Inclina());
    }
    @Override
    public boolean isFinished() {
        return false;
    }
    @Override
    public void end(boolean interrupted) {

    }
}
