package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.TurretSubsystem;

public class CommandTurret extends Command{
    private TurretSubsystem turret = RobotContainer.turretSubsystem;
    private XboxController xbox = new XboxController(0);

    public CommandTurret() {
        addRequirements(turret);
    }

    @Override
    public void initialize() {

    }
    @Override
    public void execute() {
        if(xbox.getRightTriggerAxis() > 0.1) {
            turret.SetPose_Inclina(xbox.getRightTriggerAxis() * (-10));
        } else {
            turret.SetPose_Inclina(0);
        }
    }
    @Override
    public boolean isFinished() {
        return false;
    }
    @Override
    public void end(boolean interrupted) {
        
    }
}
