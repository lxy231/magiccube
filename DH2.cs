using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class RobotKinematics
{
    // DH 参数结构体（θ, d, a, α）
    public struct DHParameters
    {
        public double theta; // 关节角度
        public double d;     // 连杆偏移
        public double a;     // 连杆长度
        public double alpha; // 连杆扭转角

        public DHParameters(double t, double d, double a, double alpha)
        {
            this.theta = t;
            this.d = d;
            this.a = a;
            this.alpha = alpha;
        }
    }

    // 目标位姿（位置 + 四元数）
    public struct TargetPose
    {
        public Vector3 Position;      // 目标位置
        public Quaternion Orientation; // 目标方向（四元数）
    }

    // 逆解主函数
    public List<double[]> InverseKinematics(TargetPose target, DHParameters[] dhParams)
    {
        if (dhParams.Length != 6)
        {
            throw new ArgumentException("This function only supports 6-DOF robots.");
        }

        List<double[]> solutions = new List<double[]>();

        // ----------------------------
        // 第1步：计算腕部中心位置
        // ----------------------------
        Vector3 endEffectorOffset = GetToolOffset(); // 工具偏移
        Matrix4x4 targetMatrix = MatrixFromPose(target);
        Matrix4x4 wristCenterMatrix = targetMatrix * Matrix4x4.Translate(-endEffectorOffset);
        Vector3 wristCenter = wristCenterMatrix.GetColumn(3);

        // ----------------------------
        // 第2步：求解前三关节 (θ1, θ2, θ3)
        // ----------------------------
        double theta1_1 = Math.Atan2(wristCenter.y, wristCenter.x);
        double theta1_2 = theta1_1 + Math.PI;

        foreach (double theta1 in new[] { theta1_1, theta1_2 })
        {
            double x = Math.Sqrt(wristCenter.x * wristCenter.x + wristCenter.y * wristCenter.y) - dhParams[0].a;
            double z = wristCenter.z - dhParams[0].d;

            double D = (x * x + z * z - dhParams[1].a * dhParams[1].a - dhParams[2].a * dhParams[2].a)
                      / (2 * dhParams[1].a * dhParams[2].a);

            if (Math.Abs(D) > 1) continue; // 无解情况

            double theta3_1 = Math.Atan2(Math.Sqrt(1 - D * D), D);
            double theta3_2 = Math.Atan2(-Math.Sqrt(1 - D * D), D);

            foreach (double theta3 in new[] { theta3_1, theta3_2 })
            {
                double theta2 = Math.Atan2(z, x) - Math.Atan2(
                    dhParams[2].a * Math.Sin(theta3),
                    dhParams[1].a + dhParams[2].a * Math.Cos(theta3)
                );

                solutions.Add(new double[] { theta1, theta2, theta3, 0, 0, 0 });
            }
        }

        // ----------------------------
        // 第3步：求解后三关节 (θ4, θ5, θ6)
        // ----------------------------
        foreach (var solution in solutions.ToArray())
        {
            Matrix4x4 R0_3 = ComputeForwardTransformation(dhParams, solution.Take(3).ToArray());
            Matrix4x4 R3_6 = R0_3.inverse * targetMatrix;

            double theta4, theta5, theta6;
            EulerAnglesFromRotation(R3_6, out theta4, out theta5, out theta6);

            solution[3] = theta4;
            solution[4] = theta5;
            solution[5] = theta6;
        }

        return FilterValidSolutions(solutions);
    }

    // 辅助函数：从位姿生成变换矩阵
    private Matrix4x4 MatrixFromPose(TargetPose pose)
    {
        return Matrix4x4.TRS(pose.Position, pose.Orientation, Vector3.one);
    }

    // 辅助函数：从旋转矩阵分解欧拉角
    private void EulerAnglesFromRotation(Matrix4x4 R, out double theta4, out double theta5, out double theta6)
    {
        theta5 = Math.Atan2(
            Math.Sqrt(R[2, 0] * R[2, 0] + R[2, 1] * R[2, 1]),
            R[2, 2]
        );

        theta4 = Math.Atan2(R[1, 2] / Math.Sin(theta5), R[0, 2] / Math.Sin(theta5));
        theta6 = Math.Atan2(R[2, 1] / Math.Sin(theta5), -R[2, 0] / Math.Sin(theta5));
    }

    // 正向运动学计算
    private Matrix4x4 ComputeForwardTransformation(DHParameters[] dhParams, double[] angles)
    {
        Matrix4x4 mat = Matrix4x4.identity;
        for (int i = 0; i < angles.Length; i++)
        {
            double theta = angles[i] + dhParams[i].theta;
            double d = dhParams[i].d;
            double a = dhParams[i].a;
            double alpha = dhParams[i].alpha;

            Matrix4x4 trans = new Matrix4x4(
                new Vector4((float)Math.Cos(theta), (float)(-Math.Sin(theta) * Math.Cos(alpha)), (float)(Math.Sin(theta) * Math.Sin(alpha)), (float)(a * Math.Cos(theta))),
                new Vector4((float)Math.Sin(theta), (float)(Math.Cos(theta) * Math.Cos(alpha)), (float)(-Math.Cos(theta) * Math.Sin(alpha)), (float)(a * Math.Sin(theta))),
                new Vector4(0, (float)Math.Sin(alpha), (float)Math.Cos(alpha), (float)d),
                new Vector4(0, 0, 0, 1)
            );
            mat *= trans;
        }
        return mat;
    }

    // 工具偏移（占位函数）
    private Vector3 GetToolOffset()
    {
        return Vector3.zero; // 根据实际工具定义
    }

    // 过滤无效解（占位函数）
    private List<double[]> FilterValidSolutions(List<double[]> solutions)
    {
        
        return solutions; // 根据关节限制过滤解


        List<double[]> validSolutions = new List<double[]>();

        foreach (var solution in solutions)
        {
            // 将关节1的角度从弧度转换为角度
            double joint1AngleInDegrees = solution[0] * (180.0 / Math.PI);

            // 检查关节1是否在限制范围内
            if (joint1AngleInDegrees >= -90 && joint1AngleInDegrees <= 90)
            {
                validSolutions.Add(solution); // 如果满足限制条件，加入有效解列表
            }
        }

         return validSolutions;
    }
}