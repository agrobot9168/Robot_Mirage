package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.subsystems.TurretSubsystem;

public class CommandTurret extends Command{
    private LimelightHelpers limelight = RobotContainer.limelightHelpers;
    private String limelightaugust = "limelight-august";
    double angle;
    double shutter;


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
        // if(xbox.getRightTriggerAxis() > 0.1 && turret.GetPose_InclinaShutter() >= -10) {
        //     turret.SetPose_Inclina(xbox.getRightTriggerAxis() * (-10));
            
        // } else {
        //     turret.SetPose_Inclina(0);
        // }
        // turret.SetPose_Inclina(-2);
        // turret.SetRPM_Shutter(1810);

        turret.SetPose_Inclina(turret.Angle());
        turret.SetRPM_Shutter(turret.VelShutter());


        SmartDashboard.putBoolean("GetTV", limelight.getTV(limelightaugust));

    }
    @Override
    public boolean isFinished() {
        return false;
    }
    @Override
    public void end(boolean interrupted) {
        turret.SetPose_Inclina(0);
        turret.SetRPM_Shutter(0);
    }
    
}
