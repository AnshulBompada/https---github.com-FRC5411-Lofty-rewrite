// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Libs;

import java.util.function.DoubleSupplier;

/** Add your docs here. */
public class ArmKinematics {
    double m_l1;
    double m_l2;
    double m_l3;
    DoubleSupplier m_stage1Radians;
    DoubleSupplier m_stage2Radians;
    DoubleSupplier m_stage3Radians;


    public ArmKinematics(double l1, double l2, double l3, DoubleSupplier stage1Radians, DoubleSupplier stage2Radians, DoubleSupplier stage3Radians) {
        m_l1 = l1;
        m_l2 = l2;
        m_l3 = l3;
        m_stage1Radians = stage1Radians;
        m_stage2Radians = stage2Radians;
        m_stage3Radians = stage3Radians;
    }
    
    public double[] inverseKinematics(double x, double y, double theta) {
        double[] output = new double[3];

        double l3 = Math.hypot(x, y);
        double thetaA = Math.acos((m_l2*m_l2-m_l1*m_l1-l3*l3)/(-2*m_l1*l3));
        double thetaB = Math.acos((l3*l3-m_l1*m_l1-m_l2*m_l2)/(-2*m_l1*m_l2));

        output[0] = (360 + Math.atan2(y, x)) + thetaA % 360;
        output[1] = (360 + output[0] + thetaB + 180) % 360;
        output[2] = (360 + theta) % 360;

        return output;
    }

    public double[] forwardKinematics() {
        double stage1Radians = m_stage1Radians.getAsDouble();
        double stage2Radians = m_stage3Radians.getAsDouble();
        double stage3Radians = m_stage3Radians.getAsDouble();

        double[] output = new double[3];
        output[0] = Math.cos(stage1Radians) * m_l1 + Math.cos(stage3Radians) * (m_l2);
        output[1] = Math.sin(stage1Radians) * m_l1 + Math.sin(stage2Radians) * (m_l2);
        output[2] = (Math.PI*2 + stage3Radians) % Math.PI*2;

        return output;
    }

    protected void screwyou() {
        inverseKinematics(m_l3, m_l2, m_l1);
        forwardKinematics();
    }
}
