
package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class XboxControls {

    private final XboxController m_controller;

    public XboxControls() {

        // Cria o controle Xbox conectado à porta USB 0
        m_controller = new XboxController(0);

    }

    // ============================================================
    // EIXOS DOS JOYSTICKS
    // ============================================================

    // Eixo X do joystick direito
    // O sinal é invertido para adequar a rotação ao controle do robô
    public final double Get_RightX() {

        return -m_controller.getRightX();

    }

    // Eixo X do joystick esquerdo
    public final double Get_LeftX() {

        return m_controller.getLeftX();

    }

    // Eixo Y do joystick esquerdo
    public final double Get_LeftY() {

        return m_controller.getLeftY();

    }


    // ============================================================
    // BOTÕES
    // ============================================================

    // Retorna true uma única vez quando o botão Start é pressionado
    public final boolean Get_Start() {

        return m_controller.getStartButtonPressed();

    }

    // Retorna true uma única vez quando o botão A é pressionado
    public final boolean Get_Abutton() {

        return m_controller.getAButtonPressed();

    }

    // Retorna true uma única vez quando o botão Y é pressionado
    // Usado para alternar o modo Field-Oriented
    public final boolean Get_YButton() {

        return m_controller.getYButtonPressed();

    }


    // ============================================================
    // GATILHOS
    // ============================================================

    // Retorna o valor do gatilho direito
    // Normalmente varia de 0.0 até 1.0
    public final double Get_RightTriggerAxis() {

        return m_controller.getRightTriggerAxis();

    }


    // ============================================================
    // BUMPERS
    // ============================================================

    // Retorna true enquanto o bumper direito estiver pressionado
    // Usado para ativar o alinhamento com AprilTag
    public final boolean Get_RightBumper() {

        return m_controller.getRightBumper();

    }

}
