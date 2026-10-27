package frc.robot.SwerveDrive;
import com.ctre.phoenix6.controls.DutyCycleOut;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.GarraConfig;
import frc.robot.XboxControls;

public class SwerveKinematics {

    private final FowardKinematics forward = new FowardKinematics();
    private static SwerveConfig swerveConfig = new SwerveConfig();
    private final XboxControls controls = new XboxControls();
    public static final ContollerPosition contollerPositionFL = new ContollerPosition(swerveConfig, swerveConfig.drive_motorFL);
    private static final ContollerPosition contollerPositionFR = new ContollerPosition(swerveConfig, swerveConfig.drive_motorFR);
    private static final ContollerPosition contollerPositionBL = new ContollerPosition(swerveConfig, swerveConfig.drive_motorBL);
    private static final ContollerPosition contollerPositionBR = new ContollerPosition(swerveConfig, swerveConfig.drive_motorBR);
    
    //private final GarraConfig motor = new GarraConfig();

    // Posição dos módulos em relação ao centro do robô
    //
    //                  FRENTE
    //
    //             FL          FR
    //              \          /
    //               \  ROBÔ  /
    //               /        \
    //              BL         BR
    //
    // X = frente / trás
    // Y = esquerda / direita

    private static final double L = 0.28128;
    private static final double W = 0.28128;

    // ============================================================
    // CONFIGURAÇÕES
    // ============================================================

    private static final double DEADBAND = 0.05;

    // Velocidade angular máxima
    private static final double MAX_ANGULAR_SPEED = 1.0;
    private static final double MAX_WHELL_SPEED_MPS = 4.5;

    // ÂNGULOS DESEJADOS
    private double targetAngleFL = 0;
    private double targetAngleFR = 0;
    private double targetAngleBL = 0;
    private double targetAngleBR = 0;

    // VELOCIDADES DOS MÓDULOS
    private double speedFL = 0;
    private double speedFR = 0;
    private double speedBL = 0;
    private double speedBR = 0;

    public SwerveKinematics() {

        // PID CONTINUOUS
        swerveConfig.pidFL.enableContinuousInput(0, 360);
        swerveConfig.pidFR.enableContinuousInput(0, 360);
        swerveConfig.pidBL.enableContinuousInput(0, 360);
        swerveConfig.pidBR.enableContinuousInput(0, 360);

        // TOLERÂNCIA
        swerveConfig.pidFL.setTolerance(1.0);
        swerveConfig.pidFR.setTolerance(1.0);
        swerveConfig.pidBL.setTolerance(1.0);
        swerveConfig.pidBR.setTolerance(1.0);
    }

    //OPTIMIZATION AND COMPESATION
    private double[] optimizeAndCompensate(
        double currentAngle,
        double targetAngle,
        double speed) {

    // Se não há movimento significativo, mantém o estado
    if (speed <= DEADBAND) {
        return new double[] { targetAngle, speed };
    }

    // Menor diferença angular entre a roda e o alvo
    double angleError = MathUtil.inputModulus(
            targetAngle - currentAngle,
            -180,
            180
    );

    // OTIMIZAÇÃO DO MÓDULO
    // Se o erro passar de 90°, inverte o sentido do drive
    // e gira o módulo 180° para usar o caminho mais curto.
    if (Math.abs(angleError) > 90) {

        targetAngle = MathUtil.inputModulus(
                targetAngle + 180,
                0,
                360
        );

        speed = -speed;

        // Recalcula o erro após alterar o alvo
        angleError = MathUtil.inputModulus(
                targetAngle - currentAngle,
                -180,
                180
        );
    }

    // COMPENSAÇÃO COSSENOIDAL
    double compensation = Math.cos(
            Math.toRadians(angleError)
    );

    speed *= compensation;

    return new double[] { targetAngle, speed };
}
    //OPTIMIZATION AND COMPESATION

                //VÁRIAVEIS
        double vx = 0;
        double vy = 0;
        double omega;

        boolean giro = false;
        boolean condition, condition2 = false;

        double zeroYaw = pigeon();
        
        
        double altTag = 0.40315;
        double altCam = 0.523;

        double deltaAlt = altTag - altCam;
        double distance = 0;


        double distanceDesejada = 0;

        boolean driveRobotOriented = false;
        boolean driveFieldOriented = true;
                //VÁRIAVEIS

                //FOR PRINTS
        private double p_vx, p_vy, p_omega, p_targetAngleFl, p_targetAngleFR, p_targetAngleBR, p_targetAngleBL,
                       p_speedFL, p_speedFR, p_speedBL, p_speedBR, p_angleFL, p_angleFR, p_angleBL, p_angleBR = 0;
                //FOR PRINTS
        boolean testes = false;
    // PERIODIC
    public void periodic() {

        if(controls.Get_Start()) {
                zeroYaw = pigeon();
        }

        // if(controls.Get_Abutton()) testes = !testes;
        // if(testes) {
        //         contollerPositionFL.setMPS(4.9);
        //         contollerPositionFR.setMPS(4.9);
        //         contollerPositionBL.setMPS(4.9);
        //         contollerPositionBR.setMPS(4.9);
        // } else {
        //         contollerPositionFL.setMPS(0);
        //         contollerPositionFR.setMPS(0);
        //         contollerPositionBL.setMPS(0);
        //         contollerPositionBR.setMPS(0);
        // }

        

        vx = 0.0;
        vy = 0.0;
        omega = 0.0;


        // // ========================================================
        // // JOYSTICK
        // // ========================================================

        // /*
        //  * LEFT Y  -> VX
        //  * LEFT X  -> VY
        //  * RIGHT X -> OMEGA
        //  */
        if(controls.Get_YButton()) driveFieldOriented = !driveFieldOriented;

                if(driveFieldOriented){
                vx =
                MathUtil.applyDeadband(
                        controls.Get_LeftY(),
                        DEADBAND
                );

                vy =
                MathUtil.applyDeadband(
                        controls.Get_LeftX(),
                        DEADBAND
                );
                omega =
                MathUtil.applyDeadband(
                        controls.Get_RightX(),
                        DEADBAND
                ) * MAX_ANGULAR_SPEED;
                

		double[] velocidades = DriveFieldOriented(vx, vy, omega);

		vx = velocidades[0];
		vy = velocidades[1];
		omega = velocidades[2];

        } else if(!driveFieldOriented) {
                vx = forward.PID_X();
                vy = forward.PID_Y();
                omega = 0.0;
        }

        
                if(driveRobotOriented) {
                vx =
                MathUtil.applyDeadband(
                        controls.Get_LeftY(),
                        DEADBAND
                );

                vy =
                MathUtil.applyDeadband(
                        controls.Get_LeftX(),
                        DEADBAND
                );
                omega =
                MathUtil.applyDeadband(
                        controls.Get_RightX(),
                        DEADBAND
                ) * MAX_ANGULAR_SPEED;
                }
        


        // ========================================================
        // CINEMATICA SWERVE
        // ========================================================

        /*
         *
         * Vx módulo = VX - omega * Y
         *
         * Vy módulo = VY + omega * X
         *
         * velocidade =
         * sqrt(Vx² + Vy²)
         *
         * ângulo =
         * atan2(Vy, Vx)
         *
         */


        // ========================================================
        // FRONT LEFT
        // ========================================================

        //Invert
        // vx = vx * -1;
        // vy = vy * -1;
        omega = omega * -1;
        //Invert



        double flVx =
                vx - omega * W;

        double flVy =
                vy + omega * L;

        speedFL =
                Math.hypot(
                        flVx,
                        flVy
                );

        if (speedFL > DEADBAND) {

            targetAngleFL =
                    Math.toDegrees(
                            Math.atan2(
                                    flVy,
                                    flVx
                            )
                    );

            targetAngleFL =
                    MathUtil.inputModulus(
                            targetAngleFL,
                            0,
                            360
                    );
        }


        // ========================================================
        // FRONT RIGHT
        // ========================================================

        double frVx =
                vx + omega * W;

        double frVy =
                vy + omega * L;

        speedFR =
                Math.hypot(
                        frVx,
                        frVy
                );

        if (speedFR > DEADBAND) {

            targetAngleFR =
                    Math.toDegrees(
                            Math.atan2(
                                    frVy,
                                    frVx
                            )
                    );

            targetAngleFR =
                    MathUtil.inputModulus(
                            targetAngleFR,
                            0,
                            360
                    );
        }


        // ========================================================
        // BACK LEFT
        // ========================================================

        double blVx =
                vx - omega * W;

        double blVy =
                vy - omega * L;

        speedBL =
                Math.hypot(
                        blVx,
                        blVy
                );

        if (speedBL > DEADBAND) {

            targetAngleBL =
                    Math.toDegrees(
                            Math.atan2(
                                    blVy,
                                    blVx
                            )
                    );

            targetAngleBL =
                    MathUtil.inputModulus(
                            targetAngleBL,
                            0,
                            360
                    );
        }


        // ========================================================
        // BACK RIGHT
        // ========================================================

        double brVx =
                vx + omega * W;

        double brVy =
                vy - omega * L;

        speedBR =
                Math.hypot(
                        brVx,
                        brVy
                );

        if (speedBR > DEADBAND) {

            targetAngleBR =
                    Math.toDegrees(
                            Math.atan2(
                                    brVy,
                                    brVx
                            )
                    );

            targetAngleBR =
                    MathUtil.inputModulus(
                            targetAngleBR,
                            0,
                            360
                    );
        }


        // ========================================================
        // NORMALIZAÇÃO DAS VELOCIDADES
        // ========================================================

        /*
         * O maior valor entre as quatro rodas vira 1.0.
         *
         * Exemplo:
         *
         * FL = 0.8
         * FR = 1.2
         * BL = 0.6
         * BR = 1.0
         *
         * Depois:
         *
         * FL = 0.66
         * FR = 1.00
         * BL = 0.50
         * BR = 0.83
         *
         * Assim mantemos a proporção entre as rodas.
         */

        double maxSpeed =
                Math.max(
                        Math.max(speedFL, speedFR),
                        Math.max(speedBL, speedBR)
                );

        if (maxSpeed > 1.0) {

            speedFL /= maxSpeed;
            speedFR /= maxSpeed;
            speedBL /= maxSpeed;
            speedBR /= maxSpeed;
        }


        // ========================================================
        // LEITURA DOS CANCODERS
        // ========================================================

        double angleFL =
                getAngle(
                        swerveConfig.canFL,
                        swerveConfig.offsetFL
                );

        double angleFR =
                getAngle(
                        swerveConfig.canFR,
                        swerveConfig.offsetFR
                );

        double angleBL =
                getAngle(
                        swerveConfig.canBL,
                        swerveConfig.offsetBL
                );

        double angleBR =
                getAngle(
                        swerveConfig.canBR,
                        swerveConfig.offsetBR
                );
        // ========================================================
	// OTIMIZAÇÃO DOS MÓDULOS + COSINE COMPENSATION
	// ========================================================

	double[] stateFL = optimizeAndCompensate(
        angleFL, targetAngleFL, speedFL
	);
	targetAngleFL = stateFL[0];
	speedFL = stateFL[1];

	double[] stateFR = optimizeAndCompensate(
        angleFR, targetAngleFR, speedFR
	);
	targetAngleFR = stateFR[0];
	speedFR = stateFR[1];

	double[] stateBL = optimizeAndCompensate(
        angleBL, targetAngleBL, speedBL
	);
	targetAngleBL = stateBL[0];
	speedBL = stateBL[1];

	double[] stateBR = optimizeAndCompensate(
        angleBR, targetAngleBR, speedBR
	);
	targetAngleBR = stateBR[0];
	speedBR = stateBR[1];

        // ========================================================
	// OTIMIZAÇÃO DOS MÓDULOS + COSINE COMPENSATION
	// ========================================================
        if(driveFieldOriented) {
                speedFL = MathUtil.clamp(speedFL * MAX_WHELL_SPEED_MPS, 
                                -MAX_WHELL_SPEED_MPS, 
                                MAX_WHELL_SPEED_MPS);
                speedFR = MathUtil.clamp(speedFR * MAX_WHELL_SPEED_MPS, 
                                        -MAX_WHELL_SPEED_MPS, 
                                        MAX_WHELL_SPEED_MPS);
                speedBL = MathUtil.clamp(speedBL * MAX_WHELL_SPEED_MPS, 
                                        -MAX_WHELL_SPEED_MPS, 
                                        MAX_WHELL_SPEED_MPS);
                speedBR = MathUtil.clamp(speedBR * MAX_WHELL_SPEED_MPS, 
                                        -MAX_WHELL_SPEED_MPS, 
                                        MAX_WHELL_SPEED_MPS);
        } else if(!driveFieldOriented) {
                speedFL = MathUtil.clamp(speedFL * 2, 
                                -2, 
                                2);
                speedFR = MathUtil.clamp(speedFR * 2, 
                                        -2, 
                                        2);
                speedBL = MathUtil.clamp(speedBL * 2, 
                                        -2, 
                                        2);
                speedBR = MathUtil.clamp(speedBR * 2, 
                                        -2, 
                                        2);
        }
        


        // ========================================================
        // PID DOS ÂNGULOS
        // ========================================================

        double outputFLValue =
                swerveConfig.pidFL.calculate(
                        angleFL,
                        targetAngleFL
                );

        double outputFRValue =
                swerveConfig.pidFR.calculate(
                        angleFR,
                        targetAngleFR
                );

        double outputBLValue =
                swerveConfig.pidBL.calculate(
                        angleBL,
                        targetAngleBL
                );

        double outputBRValue =
                swerveConfig.pidBR.calculate(
                        angleBR,
                        targetAngleBR
                );


        // ========================================================
        // LIMITAR OUTPUT DOS MOTORES DE ANGLE
        // ========================================================

        outputFLValue =
                MathUtil.clamp(
                        outputFLValue,
                        -0.5,
                        0.5
                );

        outputFRValue =
                MathUtil.clamp(
                        outputFRValue,
                        -0.5,
                        0.5
                );

        outputBLValue =
                MathUtil.clamp(
                        outputBLValue,
                        -0.5,
                        0.5
                );

        outputBRValue =
                MathUtil.clamp(
                        outputBRValue,
                        -0.5,
                        0.5
                );


        // ========================================================
        // MOTORES DE ANGLE
        // ========================================================

        // swerveConfig.motorFL.setControl(
        //         swerveConfig.outputFL.withOutput(
        //                 outputFLValue
        //         )
        // );

        // swerveConfig.motorFR.setControl(
        //         swerveConfig.outputFR.withOutput(
        //                 outputFRValue
        //         )
        // );

        // swerveConfig.motorBL.setControl(
        //         swerveConfig.outputBL.withOutput(
        //                 outputBLValue
        //         )
        // );

        // swerveConfig.motorBR.setControl(
        //         swerveConfig.outputBR.withOutput(
        //                 outputBRValue
        //         )
        // );





        swerveConfig.motorFL.set(outputFLValue);

        swerveConfig.motorFR.set(outputFRValue);

        swerveConfig.motorBL.set(outputBLValue);

        swerveConfig.motorBR.set(outputBRValue);







        // ========================================================
        // MOTORES DE DRIVE
        // ========================================================

        // swerveConfig.drive_motorFL.set(speedFL);
        // swerveConfig.drive_motorFR.set(speedFR);
        // swerveConfig.drive_motorBL.set(speedBL);
        // swerveConfig.drive_motorBR.set(speedBR);
        contollerPositionFL.setMPS(speedFL);
        contollerPositionFR.setMPS(speedFR);
        contollerPositionBL.setMPS(speedBL);
        contollerPositionBR.setMPS(speedBR);


        // ========================================================
        // FOR DASHBOARD
        vx = p_vx;
        vy = p_vy;
        omega = p_omega;
        targetAngleFL = p_targetAngleFl;
        targetAngleFR = p_targetAngleFR;
        targetAngleBL = p_targetAngleBL;
        targetAngleBR = p_targetAngleBR;
        speedFL = p_speedFL;
        speedFR = p_speedFR;
        speedBL = p_speedBL;
        speedBR = p_speedBR;
        angleFL = p_angleFL;
        angleFR = p_angleFR;
        angleBL = p_angleBL;
        angleBR = p_angleBR;
        // ========================================================
    }

    // ============================================================
    // CANcoder → GRAUS
    // ============================================================

    private static double getAngle(CANcoder cancoder, double offset) {
    
            double rotations =
                    cancoder
                            .getAbsolutePosition()
                            .getValueAsDouble();
    
            double degrees =
                    rotations * 360.0;
    
            degrees -= offset;
    
            return MathUtil.inputModulus(
                    degrees,
                    0,
                    360
            );
        }
        double pigeon() {
            return swerveConfig.pigeon2.getYaw().getValueAsDouble();
        }
    
        public double[] DriveFieldOriented(double vx, double vy, double omega) {
    
        // Orientação atual do Pigeon, em graus
        double yaw = pigeon();
    
        // Orientação relativa à referência salva ao apertar Start
        double heading = Math.toRadians(yaw - zeroYaw);
    
        // Transformação Field-Oriented para Robot-Oriented
        double robotVx = vx * Math.cos(heading) + vy * Math.sin(heading);
        double robotVy = -vx * Math.sin(heading) + vy * Math.cos(heading);
    
        // Retorna: Vx, Vy, omega
        return new double[] { robotVx, robotVy, omega };
        }
        
    
        public void PrintsSwerveKnematics() {
            SmartDashboard.putNumber("RPS/FL", contollerPositionFL.getMPS());
            SmartDashboard.putNumber("RPS/FR", contollerPositionFR.getMPS());
            SmartDashboard.putNumber("RPS/BL", contollerPositionBL.getMPS());
            SmartDashboard.putNumber("RPS/BR", contollerPositionBR.getMPS());
    
            SmartDashboard.putNumber("Position/Position_FL", contollerPositionFL.GetPosition());
    
    
                            SmartDashboard.putNumber(
                    "VX",
                    p_vx
            );
    
            SmartDashboard.putNumber(
                    "VY",
                    p_vy
            );
    
            SmartDashboard.putNumber(
                    "Omega",
                    p_omega
            );
    
    
            SmartDashboard.putNumber(
                    "FL Target Angle",
                    p_targetAngleFl
            );
    
            SmartDashboard.putNumber(
                    "FR Target Angle",
                    p_targetAngleFR
            );
    
            SmartDashboard.putNumber(
                    "BL Target Angle",
                    p_targetAngleBL
            );
    
            SmartDashboard.putNumber(
                    "BR Target Angle",
                    p_targetAngleBR
            );
    
    
            SmartDashboard.putNumber(
                    "FL Speed",
                    p_speedFL
            );
    
            SmartDashboard.putNumber(
                    "FR Speed",
                    p_speedFR
            );
    
            SmartDashboard.putNumber(
                    "BL Speed",
                    p_speedBL
            );
    
            SmartDashboard.putNumber(
                    "BR Speed",
                    p_speedBR
            );
    
    
            SmartDashboard.putNumber(
                    "FL Angle",
                    p_angleFL
            );
    
            SmartDashboard.putNumber(
                    "FR Angle",
                    p_angleFR
            );
    
            SmartDashboard.putNumber(
                    "BL Angle",
                    p_angleBL
            );
    
            SmartDashboard.putNumber(
                    "BR Angle",
                    p_angleBR
            );
    
    
            SmartDashboard.putNumber(
                    "Pigeon2",
                    swerveConfig.pigeon2
                            .getYaw()
                            .getValueAsDouble()
            );
    
    }
    
    //public SwerveKinematics Garrateste() {
            
        // if(controls.Get_Abutton()) {
         // motor.set(0.3);
        // }
    
    
         //if(controls.getBButtonPressed()) {
          // motorGarra.set(-0.8);
        //}
    
        //else {
        //  motor.set(0.0);
      // }
    //}
    
    
    
    
    //         double Distance() {
    //         double TyCam = LimelightHelpers.getTY("limelight-august");
    //         return distance = deltaAlt / Math.tan(Math.toRadians(TyCam));
    //     }
    //         SmartDashboard.putNumber("DISTANCE", Distance());
    public static double[] DirectKinematics() {
    
        // Velocidades reais das rodas (m/s)
        double vFL = contollerPositionFL.getMPS();
        double vFR = contollerPositionFR.getMPS();
        double vBL = contollerPositionBL.getMPS();
        double vBR = contollerPositionBR.getMPS();
    
        // Ângulos reais dos módulos (graus)
        double aFL = Math.toRadians(
            getAngle(swerveConfig.canFL, swerveConfig.offsetFL)
        );
        double aFR = Math.toRadians(
            getAngle(swerveConfig.canFR, swerveConfig.offsetFR)
    );
    double aBL = Math.toRadians(
        getAngle(swerveConfig.canBL, swerveConfig.offsetBL)
    );
    double aBR = Math.toRadians(
        getAngle(swerveConfig.canBR, swerveConfig.offsetBR)
    );

    // Componentes de velocidade de cada roda
    double flVx = vFL * Math.cos(aFL);
    double flVy = vFL * Math.sin(aFL);

    double frVx = vFR * Math.cos(aFR);
    double frVy = vFR * Math.sin(aFR);

    double blVx = vBL * Math.cos(aBL);
    double blVy = vBL * Math.sin(aBL);

    double brVx = vBR * Math.cos(aBR);
    double brVy = vBR * Math.sin(aBR);

    // Cinemática direta: velocidade translacional
    double vx = (flVx + frVx + blVx + brVx) / 4.0;
    double vy = (flVy + frVy + blVy + brVy) / 4.0;

    // Velocidade angular estimada (rad/s)
    double omega = (
          (-flVx + flVy) 
        + ( frVx + frVy)
        + (-blVx - blVy)
        + ( brVx - brVy)
    ) / (4.0 * (L + W));

    return new double[] { vx, vy, omega };
}
}