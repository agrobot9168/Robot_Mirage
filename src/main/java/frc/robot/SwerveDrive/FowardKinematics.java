package frc.robot.SwerveDrive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.SwerveDrive.SwerveKinematics;

public class FowardKinematics {

private double alt = 0.40315 - 0.523;
private double mensuredX = 0.8;
private double mensuredY = 0.1;

    private PIDController pid_TX = new PIDController(0.07, 0, 0);
    private PIDController pid_TY = new PIDController(0.07, 0, 0);


    public FowardKinematics() {

    }
    public double Erro_X() {
        double ty = Math.toRadians(LimelightHelpers.getTY("limelight"));
        return Math.abs(alt / Math.tan(ty));
    }
    // public double TrueDistance() {
    //     return Math.hypot(alt, Erro_X());
    // }
    public double Erro_Y() {
        double tx = Math.toRadians(LimelightHelpers.getTX("limelight"));
        return Math.abs(Erro_X() * Math.tan(tx));
    }

    public double PID_X() {
        if(Erro_X() > mensuredX) {
            return pid_TX.calculate(mensuredX, Erro_X());
        } else {
            return 0.0;
        }
    }
    public double PID_Y() {
        if(Erro_X() > Math.abs(mensuredY)) {
            return pid_TY.calculate(mensuredY, Erro_Y());
        } else {
            return 0.0;
        }
    }
    

    public void PrintFowardKinematics() {
        SmartDashboard.putNumber("VelChassi/VX", SwerveKinematics.DirectKinematics()[0]);
        SmartDashboard.putNumber("VelChassi/VY", SwerveKinematics.DirectKinematics()[1]);
        SmartDashboard.putNumber("VelChassi/OMEGA", SwerveKinematics.DirectKinematics()[2]);

        SmartDashboard.putNumber("ERRO_X", Erro_X());
        SmartDashboard.putNumber("ERRO_Y", Erro_Y());
        SmartDashboard.putBoolean("TEMTAG", LimelightHelpers.getTV("limelight"));
    }


    // private SwerveKinematics swerveKinematics = new SwerveKinematics();
    // private SwerveConfig swerveConfig = new SwerveConfig();

    // private final ContollerPosition contollerPositionFL = new ContollerPosition(swerveConfig, swerveConfig.drive_motorFL);
    // private final ContollerPosition contollerPositionFR = new ContollerPosition(swerveConfig, swerveConfig.drive_motorFR);
    // private final ContollerPosition contollerPositionBL = new ContollerPosition(swerveConfig, swerveConfig.drive_motorBL);
    // private final ContollerPosition contollerPositionBR = new ContollerPosition(swerveConfig, swerveConfig.drive_motorBR);
    
    // public FowardKinematics() {

    // }
    
    // public double[] DirectKinematics() {

    // // Velocidades reais das rodas (m/s)
    // double vFL = contollerPositionFL.getMPS();
    // double vFR = contollerPositionFR.getMPS();
    // double vBL = contollerPositionBL.getMPS();
    // double vBR = contollerPositionBR.getMPS();

    // // Ângulos reais dos módulos (graus)
    // double aFL = Math.toRadians(
    //     swerveKinematics.getAngle(swerveConfig.canFL, swerveConfig.offsetFL)
    // );
    // double aFR = Math.toRadians(
    //     swerveKinematics.getAngle(swerveConfig.canFR, swerveConfig.offsetFR)
    // );
    // double aBL = Math.toRadians(
    //     swerveKinematics.getAngle(swerveConfig.canBL, swerveConfig.offsetBL)
    // );
    // double aBR = Math.toRadians(
    //     swerveKinematics.getAngle(swerveConfig.canBR, swerveConfig.offsetBR)
    // );

    // // Componentes de velocidade de cada roda
    // double flVx = vFL * Math.cos(aFL);
    // double flVy = vFL * Math.sin(aFL);

    // double frVx = vFR * Math.cos(aFR);
    // double frVy = vFR * Math.sin(aFR);

    // double blVx = vBL * Math.cos(aBL);
    // double blVy = vBL * Math.sin(aBL);

    // double brVx = vBR * Math.cos(aBR);
    // double brVy = vBR * Math.sin(aBR);

    // // Cinemática direta: velocidade translacional
    // double vx = (flVx + frVx + blVx + brVx) / 4.0;
    // double vy = (flVy + frVy + blVy + brVy) / 4.0;

    // // Velocidade angular estimada (rad/s)
    // double omega = (
    //       (-flVx + flVy) 
    //     + ( frVx + frVy)
    //     + (-blVx - blVy)
    //     + ( brVx - brVy)
    // ) / (4.0 * (swerveKinematics.L + swerveKinematics.W));

    // return new double[] { vx, vy, omega };
    // }

    // public void PrintFowardKinematics() {
    //     SmartDashboard.putNumber("VelChassi/VX", DirectKinematics()[0]);
    //     SmartDashboard.putNumber("VelChassi/VY", DirectKinematics()[1]);
    //     SmartDashboard.putNumber("VelChassi/TETA", DirectKinematics()[2]);
    // }
}
