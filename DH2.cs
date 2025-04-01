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
    // 正向运动学计算
    private Matrix4x4 ComputeForwardTransformation(DHParameters[] dhParams, double[] angles)
    {
        Matrix4x4 mat = Matrix4x4.identity; // 初始化为单位矩阵
        for (int i = 0; i < angles.Length; i++)
        {
            // 提取 DH 参数
            double theta = angles[i] + dhParams[i].theta;
            double d = dhParams[i].d;
            double a = dhParams[i].a;
            double alpha = dhParams[i].alpha;

            // 构造当前关节的变换矩阵
            Matrix4x4 trans = new Matrix4x4(
                new Vector4((float)Math.Cos(theta), (float)(-Math.Sin(theta) * Math.Cos(alpha)), (float)(Math.Sin(theta) * Math.Sin(alpha)), (float)(a * Math.Cos(theta))),
                new Vector4(0, (float)Math.Cos(alpha), (float)(-Math.Sin(alpha)), (float)d),
                new Vector4((float)Math.Sin(theta), (float)(Math.Cos(theta) * Math.Cos(alpha)), (float)(-Math.Cos(theta) * Math.Sin(alpha)), (float)(a * Math.Sin(theta))),
                new Vector4(0, 0, 0, 1)
            );
            mat *= trans; // 累乘变换矩阵
        }
        return mat;
    }
    // 逆解主函数
    public List<double[]> InverseKinematics(TargetPose target, DHParameters[] dhParams)
    {
        // 检查 DH 参数是否为 6 个自由度
        if (dhParams.Length != 6)
        {
            throw new ArgumentException("This function only supports 6-DOF robots.");
        }

        List<double[]> solutions = new List<double[]>();

        // ----------------------------
        // 第1步：计算腕部中心位置
        // ----------------------------
        Vector3 endEffectorOffset = GetToolOffset(); // 获取工具偏移
        Matrix4x4 targetMatrix = MatrixFromPose(target); // 将目标位姿转换为变换矩阵
        Matrix4x4 wristCenterMatrix = targetMatrix * Matrix4x4.Translate(-endEffectorOffset); // 计算腕部中心矩阵
        Vector3 wristCenter = wristCenterMatrix.GetColumn(3); // 提取腕部中心位置

        // ----------------------------
        // 第2步：求解前三关节 (θ1, θ2, θ3)
        // ----------------------------
        double theta1_1 = Math.Atan2(wristCenter.z, wristCenter.x); // θ1 的第一个解
        double theta1_2 = theta1_1 + Math.PI; // θ1 的第二个解

        foreach (double theta1 in new[] { theta1_1, theta1_2 })
        {
            // 计算腕部中心在 x-y 平面上的投影
            double x = Math.Sqrt(wristCenter.x * wristCenter.x + wristCenter.z * wristCenter.z) - dhParams[0].a;
            double y = wristCenter.y - dhParams[0].d;

            // 计算 θ3 的余弦值
            double D = (x * x + y * y - dhParams[1].a * dhParams[1].a - dhParams[2].a * dhParams[2].a)
                      / (2 * dhParams[1].a * dhParams[2].a);

            if (Math.Abs(D) > 1) continue; // 如果 D 超出 [-1, 1]，则无解

            // 计算 θ3 的两个解
            double theta3_1 = Math.Atan2(Math.Sqrt(1 - D * D), D);
            double theta3_2 = Math.Atan2(-Math.Sqrt(1 - D * D), D);

            foreach (double theta3 in new[] { theta3_1, theta3_2 })
            {
                // 计算 θ2
                double theta2 = Math.Atan2(y, x) - Math.Atan2(
                    dhParams[2].a * Math.Sin(theta3),
                    dhParams[1].a + dhParams[2].a * Math.Cos(theta3)
                );

                // 保存解 (θ1, θ2, θ3)，后三关节先初始化为 0
                solutions.Add(new double[] { theta1, theta2, theta3, 0, 0, 0 });
            }
        }

        // ----------------------------
        // 第3步：求解后三关节 (θ4, θ5, θ6)
        // ----------------------------
        foreach (var solution in solutions.ToArray())
        {
            // 计算前三关节的正向变换矩阵
            Matrix4x4 R0_3 = ComputeForwardTransformation(dhParams, solution.Take(3).ToArray());
            // 计算后三关节的变换矩阵
            Matrix4x4 R3_6 = R0_3.inverse * targetMatrix;

            // 从旋转矩阵分解后三关节的欧拉角
            double theta4, theta5, theta6;
            EulerAnglesFromRotation(R3_6, out theta4, out theta5, out theta6);

            // 保存后三关节的解
            solution[3] = theta4;
            solution[4] = theta5;
            solution[5] = theta6;
        }

        // 返回过滤后的有效解
        return FilterValidSolutions(solutions);
    }

    // 辅助函数：从位姿生成变换矩阵
    private Matrix4x4 MatrixFromPose(TargetPose pose)
    {
        // 使用 Unity 的 TRS 函数生成变换矩阵
        return Matrix4x4.TRS(pose.Position, pose.Orientation, Vector3.one);
    }

    // 辅助函数：从旋转矩阵分解欧拉角
    private void EulerAnglesFromRotation(Matrix4x4 R, out double theta4, out double theta5, out double theta6)
    {
        // 计算 θ5
        theta5 = Math.Atan2(
            Math.Sqrt(R[0, 2] * R[0, 2] + R[2, 2] * R[2, 2]),
            R[1, 2]
        );

        // 计算 θ4 和 θ6
        theta4 = Math.Atan2(R[1, 0] / Math.Sin(theta5), R[1, 1] / Math.Sin(theta5));
        theta6 = Math.Atan2(R[0, 2] / Math.Sin(theta5), -R[2, 2] / Math.Sin(theta5));
    }



    // 过滤无效解（根据每个关节的角度限制）
    private List<double[]> FilterValidSolutions(List<double[]> solutions)
    {
        List<double[]> validSolutions = new List<double[]>();

        // 定义每个关节的角度限制（单位：度）
        double[][] jointLimits = new double[][]
        {
            new double[] { -90, 90 },    // 关节1限制：-90° ~ 90°
            new double[] { -45, 135 },  // 关节2限制：-45° ~ 135°
            new double[] { -120, 120 }, // 关节3限制：-120° ~ 120°
            new double[] { -180, 180 }, // 关节4限制：-180° ~ 180°
            new double[] { -90, 90 },   // 关节5限制：-90° ~ 90°
            new double[] { -360, 360 }  // 关节6限制：-360° ~ 360°
        };

        foreach (var solution in solutions)
        {
            bool isValid = true;

            for (int i = 0; i < solution.Length; i++)
            {
                // 将关节角度从弧度转换为角度
                double jointAngleInDegrees = solution[i] * (180.0 / Math.PI);

                // 检查关节是否在限制范围内
                if (jointAngleInDegrees < jointLimits[i][0] || jointAngleInDegrees > jointLimits[i][1])
                {
                    isValid = false;
                    break; // 如果某个关节超出限制，直接跳过该解
                }
            }

            if (isValid)
            {
                validSolutions.Add(solution); // 如果所有关节都满足限制条件，加入有效解列表
            }
        }

        return validSolutions; // 返回有效解
    }

    // 工具偏移（占位函数）
    private Vector3 GetToolOffset()
    {
        return Vector3.zero; // 返回工具偏移，实际实现需根据工具定义
    }
}