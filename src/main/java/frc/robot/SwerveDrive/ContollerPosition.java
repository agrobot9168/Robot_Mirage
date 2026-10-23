package frc.robot.SwerveDrive;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;

public class ContollerPosition {
    private static final double kP = 0.0001; // Exemplo inicial
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kV = 12.0 / 5676.0; // Estimativa inicial para NEO
    private static final double maxAcceleration = 5600 * 4; // RPM/s

    private SparkMaxConfig config;

    private SparkClosedLoopController controlador;
    private RelativeEncoder encoder;

    public ContollerPosition(SwerveConfig sw_cfg, SparkMax motor) {

        config = new SparkMaxConfig();

        controlador = motor.getClosedLoopController();
        encoder = motor.getEncoder();


        config.closedLoop.p(kP).i(kI).d(kD);
        config.closedLoop.feedForward.kV(kV);
        config.closedLoop.maxMotion.maxAcceleration(maxAcceleration);
        motor.configure(config, com.revrobotics.ResetMode.kResetSafeParameters, 
                                                com.revrobotics.PersistMode.kNoPersistParameters);
    }

    public void setRPM(double RPM) {
        controlador.setSetpoint(RPM, ControlType.kMAXMotionVelocityControl);
    }

    public void setMPS(double mps) {
        double rpm = ((mps / 0.319186) * 6.12);
        controlador.setSetpoint(rpm, ControlType.kMAXMotionVelocityControl);
    }
    public void setMPSAlternative(double mps) {
        double RPM = MathUtil.clamp(mps, -5676.0, 5676.0);
        controlador.setSetpoint(RPM, ControlType.kMAXMotionVelocityControl);
    }

    public double getRPM() {
        return encoder.getVelocity();
    }
    public double getMPS() {
        double rps = encoder.getVelocity() / 60;
        double m_s = (rps / 6.12) * 0.319186;
        return m_s;
    }
}
