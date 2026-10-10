package frc.robot.SwerveDrive;
import frc.robot.LimelightHelpers;
import frc.robot.SwerveDrive.AprilTagYaw;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveKinematics {
        double kOmega = 0.2;
        double valor = 0;
        double omega;

        private final AprilTagYaw aprilTagYaw =
        new AprilTagYaw("limelightaugust");
        private boolean aprilTagAlignment = false;

        private PIDController pidLateral = new PIDController(0.03, 0, 0);
        private PIDController pidFrontal = new PIDController(0.5, 0, 0);

        private PIDController pidYaw = new PIDController(0.04, 0, 0);

    private SwerveConfig swerveConfig = new SwerveConfig();

    

    // ============================================================
    // GEOMETRIA DO ROBÔ
    // ============================================================

    // Posição dos módulos em relação ao centro do robô
    //
    //                  FRENTE
    //
    //             FL          FR
    //              \          /
    //               \  ROBÔ  /
    //              /         \
    //             BL          BR
    //
    // X = frente / trás
    // Y = esquerda / direita

    private static final double L = 0.28128;
    private static final double W = 0.28128;

    // ============================================================
    // CONFIGURAÇÕES
    // ============================================================

    private static final double DEADBAND = 0.1;

    // Velocidade angular máxima
    private static final double MAX_ANGULAR_SPEED = 1.0;

    // ============================================================
    // ÂNGULOS DESEJADOS
    // ============================================================

    private double targetAngleFL = 0;
    private double targetAngleFR = 0;
    private double targetAngleBL = 0;
    private double targetAngleBR = 0;

    // ============================================================
    // VELOCIDADES DOS MÓDULOS
    // ============================================================

    private double speedFL = 0;
    private double speedFR = 0;
    private double speedBL = 0;
    private double speedBR = 0;

    public SwerveKinematics() {

        // ========================================================
        // PID CONTINUOUS
        // ========================================================

        swerveConfig.pidFL.enableContinuousInput(0, 360);
        swerveConfig.pidFR.enableContinuousInput(0, 360);
        swerveConfig.pidBL.enableContinuousInput(0, 360);
        swerveConfig.pidBR.enableContinuousInput(0, 360);

        pidYaw.enableContinuousInput(-180, 180);
        pidYaw.setTolerance(2.0);

        // ========================================================
        // TOLERÂNCIA
        // ========================================================

        swerveConfig.pidFL.setTolerance(1.0);
        swerveConfig.pidFR.setTolerance(1.0);
        swerveConfig.pidBL.setTolerance(1.0);
        swerveConfig.pidBR.setTolerance(1.0);
    }

    // ============================================================
    // SET ANGLE
    // ============================================================

    public void setAngleFL(double degrees) {
        targetAngleFL =
                MathUtil.inputModulus(degrees, 0, 360);
    }

    public void setAngleFR(double degrees) {
        targetAngleFR =
                MathUtil.inputModulus(degrees, 0, 360);
    }

    public void setAngleBL(double degrees) {
        targetAngleBL =
                MathUtil.inputModulus(degrees, 0, 360);
    }

    public void setAngleBR(double degrees) {
        targetAngleBR =
                MathUtil.inputModulus(degrees, 0, 360);
    }

    // ============================================================
    // PERIODIC
    // ============================================================
        //where I am modification
        double vx = 0;
        double vy = 0;
        boolean giro = false;
        boolean condition, condition2 = false;

        double zeroYaw = pigeon();
        
        
        double altTag = 0.40315;
        double altCam = 0.523;

        double deltaAlt = altTag - altCam;
        double distance = 0;

        
        

        double distanceDesejada = 0;
        //where I am modification

    public void periodic() {
        vx = 0.0;
        vy = 0.0;
        omega = 0.0;


        SmartDashboard.putNumber("DISTANCE", Distance());
                                //where I am modification
        
        double Yaw = pigeon();

        if(swerveConfig.xboxController.getAButtonPressed()) {
                condition = !condition;
                condition2 = false;
        }

        if(swerveConfig.xboxController.getBButtonPressed()) {
                vx = 0.0;
                vy = 0.0;
                condition2 = !condition2;
                condition = false;
                pidYaw.reset();
        }

        if(swerveConfig.xboxController.getStartButtonPressed()){
                zeroYaw = pigeon();
                pidYaw.reset();         
        }
        SmartDashboard.putBoolean("CONDITION", condition);
        SmartDashboard.putBoolean("CONDITION2", condition2);

        if(condition2){
                double Yawtarget = MathUtil.inputModulus(zeroYaw + 180, -180, 180);
                omega = pidYaw.calculate(Yaw, Yawtarget);
                omega = MathUtil.clamp(omega, -0.2, 0.2);
        
                if(pidYaw.atSetpoint()) {
                        omega = 0;
                        condition2 = false;
                        pidYaw.reset(); 
                }
        }
        
                                //where I am modification

        // ========================================================
        // JOYSTICK
        // ========================================================

        /*
         * LEFT Y  -> VX
         * LEFT X  -> VY
         * RIGHT X -> OMEGA
         */

        
                if(condition) {
                vx =
                MathUtil.applyDeadband(
                        swerveConfig.xboxController.getLeftY(),
                        DEADBAND
                );

                vy =
                MathUtil.applyDeadband(
                        swerveConfig.xboxController.getLeftX(),
                        DEADBAND
                );

        

                if(swerveConfig.xboxController.getRightTriggerAxis() >= 0.1 &&
                !LimelightHelpers.getTV("limelight-august")) {
                                 omega =
                MathUtil.applyDeadband(
                        0.5 * kOmega,
                        DEADBAND
                ) * MAX_ANGULAR_SPEED;
                } else if(swerveConfig.xboxController.getRightTriggerAxis() <0.1){
                         omega =
                MathUtil.applyDeadband(
                        swerveConfig.xboxController.getRightX(),
                        DEADBAND
                ) * MAX_ANGULAR_SPEED;
                }if(LimelightHelpers.getTV("limelight-august") &&
                swerveConfig.xboxController.getRightTriggerAxis() >= 0.1) {
                        if(LimelightHelpers.getTX("limelight-august") > 1 ||
                           LimelightHelpers.getTX("limelight-august") < -1
                           ){

                                if((LimelightHelpers.getTX("limelight-august") < 20 &&
                                LimelightHelpers.getTX("limelight-august") > 15) ||
                                (LimelightHelpers.getTX("limelight-august") > -20 &&
                                LimelightHelpers.getTX("limelight-august") < -15)) {
                                        kOmega = 0.05;
                                }
                                if((LimelightHelpers.getTX("limelight-august") <= 15 &&
                                LimelightHelpers.getTX("limelight-august") > 5) || 
                                (LimelightHelpers.getTX("limelight-august") >= -15 &&
                                LimelightHelpers.getTX("limelight-august") < -5)) {
                                        kOmega = 0.02;
                                }
                         omega =
                MathUtil.applyDeadband(
                        kOmega * LimelightHelpers.getTX("limelight-august"),
                        DEADBAND
                ) * MAX_ANGULAR_SPEED;
                }
        }

                } else {
                        vx = 0;
                        vy = 0;
                }
        //         else if(false) {
        //                 //where I am modification
        // omega = 0;
        // double TxCam = LimelightHelpers.getTX("limelight-august");

        // double distanceLateral =
        // Distance() * Math.sin(Math.toRadians(TxCam));

        // double distanceFrontal =
        // Distance() * Math.cos(Math.toRadians(TxCam));

        // //PID Lateral
        //         vy = pidLateral.calculate(distanceLateral, 0);
        // //PID Frontal
        //         vx = pidFrontal.calculate(distanceFrontal, distanceDesejada);

        // //Vel Limit
        //         vy = MathUtil.clamp(vy, -0.5, 0.5);
        //         vx = MathUtil.clamp(vx, -0.5, 0.5);


        
        //                 //where I am modification
        //         }
                
                
        // if(swerveConfig.xboxController.getRightTriggerAxis() >= 0.1) {
        //         aprilTagAlignment = true;
        // } else {
        //         aprilTagAlignment = false;
        // }
        //         double omega;

        // if (aprilTagAlignment) {

        // omega = aprilTagYaw.getOmega();

        // } else {

        // omega =
        //         MathUtil.applyDeadband(
        //                 swerveConfig.xboxController.getRightX(),
        //                 DEADBAND
        //         ) * MAX_ANGULAR_SPEED;
        // }


        // ========================================================
        // CINEMÁTICA SWERVE
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

        swerveConfig.drive_motorFL.set(speedFL * -1);
        swerveConfig.drive_motorFR.set(speedFR);
        swerveConfig.drive_motorBL.set(speedBL);
        swerveConfig.drive_motorBR.set(speedBR);


        // ========================================================
        // DASHBOARD
        // ========================================================

        SmartDashboard.putNumber(
                "VX",
                vx
        );

        SmartDashboard.putNumber(
                "VY",
                vy
        );

        SmartDashboard.putNumber(
                "Omega",
                omega
        );


        SmartDashboard.putNumber(
                "FL Target Angle",
                targetAngleFL
        );

        SmartDashboard.putNumber(
                "FR Target Angle",
                targetAngleFR
        );

        SmartDashboard.putNumber(
                "BL Target Angle",
                targetAngleBL
        );

        SmartDashboard.putNumber(
                "BR Target Angle",
                targetAngleBR
        );


        SmartDashboard.putNumber(
                "FL Speed",
                speedFL
        );

        SmartDashboard.putNumber(
                "FR Speed",
                speedFR
        );

        SmartDashboard.putNumber(
                "BL Speed",
                speedBL
        );

        SmartDashboard.putNumber(
                "BR Speed",
                speedBR
        );


        SmartDashboard.putNumber(
                "FL Angle",
                angleFL
        );

        SmartDashboard.putNumber(
                "FR Angle",
                angleFR
        );

        SmartDashboard.putNumber(
                "BL Angle",
                angleBL
        );

        SmartDashboard.putNumber(
                "BR Angle",
                angleBR
        );


        SmartDashboard.putNumber(
                "Pigeon2",
                swerveConfig.pigeon2
                        .getYaw()
                        .getValueAsDouble()
        );
    }


    // ============================================================
    // CANcoder → GRAUS
    // ============================================================

    private double getAngle(
            CANcoder cancoder,
            double offset
    ) {

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
    double Distance() {
        double TyCam = LimelightHelpers.getTY("limelight-august");
        return distance = deltaAlt / Math.tan(Math.toDegrees(TyCam));
    }
}