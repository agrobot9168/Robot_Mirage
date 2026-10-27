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
    private static final double MAX_RPM = 5676.0;


    private double Vi = 0.0;
    private double Vf = 0.0;
    private double positionI = 0.0;
    private double positionF = 0.0;

    

    double wheelCircunference = 0.319186;
    double velFactor = wheelCircunference / 6.12 / 60;

    private SparkMaxConfig config;

    private SparkClosedLoopController controlador;
    private static RelativeEncoder encoder;

    public ContollerPosition(SwerveConfig sw_cfg, SparkMax motor) {

        config = new SparkMaxConfig();

        controlador = motor.getClosedLoopController();
        encoder = motor.getEncoder();

        motor.getEncoder().setPosition(0);
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
        double rpm = ((mps / 0.319186) * 6.12 * 60);
        rpm = MathUtil.clamp(rpm, -MAX_RPM, MAX_RPM);
        controlador.setSetpoint(rpm, ControlType.kMAXMotionVelocityControl);
    }

    public double getRPM() {
        return encoder.getVelocity();
    }
    public double getMPS() {
        double rps = encoder.getVelocity() / 60;
        double m_s = (rps / 6.12) * 0.319186;
        return m_s;
    }

    public double GetPosition() {
    Vf = getMPS();

    positionI = ((Vi + Vf) / 2.0) * 0.02;
    positionF += positionI;

    Vi = Vf;

    return positionF;
    }
}
