package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;

public class CommandIntake extends Command{
    private final IntakeSubsystem intake = RobotContainer.intakeSubsystem;

    public CommandIntake() {
        addRequirements(intake);
    }
    @Override
    public void initialize() {

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
