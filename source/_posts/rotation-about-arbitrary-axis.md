---
title: 绕任意轴旋转
comments: false
date: 2022-04-09 10:29:00
tags:
---

注：以下基于**左手坐标系**。

要让点 P 绕过 AB 两点的轴（以下称 AB）旋转 $\theta$ 得到 P'，可按以下步骤进行：

1）将 A 点平移到原点

$$
T =
\begin{bmatrix}
1 & 0 & 0 & -A_x \\
0 & 1 & 0 & -A_y \\
0 & 0 & 1 & -A_z \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

2）将 AB 绕 Y 轴旋转 $\theta_y$ 度，使其位于 XY 平面上

其中，$\theta_y$ 为 AB 在 XZ 平面上的投影 ABproj $(B_x, 0, B_z)$ 与 X 轴 $(1, 0, 0)$ 的夹角。

这里需要注意，ABproj 与 X 轴的左右关系决定了顺时针旋转还是逆时针旋转。当 ABproj 位于 X 轴的**右**侧时，须将旋转角度取反。

$$
R_y =
\begin{bmatrix}
\cos(\theta_y) & 0 & \sin(\theta_y) & 0 \\
0 & 1 & 0 & 0 \\
-\sin(\theta_y) & 0 & \cos(\theta_y) & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

3）将 AB 绕 Z 轴旋转 $\theta_z$ 度，使其位于 X 轴上

其中，$\theta_z$ 为 AB 与 AB 在 XZ 平面上的投影 ABproj $(B_x, 0, B_z)$ 的夹角。

同样需要注意左右关系，当  AB 位于 ABproj 轴的**左**侧时，须将旋转角度取反。

$$
R_z =
\begin{bmatrix}
\cos(\theta_z) & -\sin(\theta_z) & 0 & 0 \\
\sin(\theta_z) & \cos(\theta_z) & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

4）绕 X 轴旋转 $\theta_x$ 度

此时，AB 与 X 轴对齐，所以绕 X 轴旋转可视为绕 AB 旋转。 $\theta_x$ 即为 P 点需要旋转的角度 $\theta$。

$$
R_x =
\begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & \cos(\theta) & -\sin(\theta) & 0 \\
0 & \sin(\theta) & \cos(\theta) & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

5）最后，做 3、 2、 1 的逆操作

将 AB 与 P 移回原来的相对位置。

旋转矩阵的逆为其转置矩阵，平移矩阵的逆为 $T(t)^{-1} = T(-t)$。

因此，最终绕任意轴旋转可表示为：

$$
P' = T(-t) \cdot R_y^T \cdot R_z^T \cdot R_x \cdot R_z \cdot R_y \cdot T \cdot P
$$

代码如下：

```cpp
// 计算 v1 和 v2 的余弦值和正弦值
// vec2.x 为余弦值，vec2.y 为正弦值
vec2 cossin(vec3 v1, vec3 v2)
{
    if (v1 == vec3.zero || v2 == vec3.zero)
    {
        return vec2(1, 0);
    }
    auto d = v1.magnitude * v2.magnitude;
    auto cos = v1.dot(v2) / d;
    auto sin = v1.cross(v2).magnitude / d;
    return vec2(cos, sin);
}

vec3 rotate(vec3 a, vec3 b, vec3 p, float angle)
{
    // 平移到原点
    auto t = mat4x4(1, 0, 0, -a.x,
                    0, 1, 0, -a.y,
                    0, 0, 1, -a.z,
                    0, 0, 0, 1);
    auto tr = mat4x4(1, 0, 0, a.x,
                     0, 1, 0, a.y,
                     0, 0, 1, a.z,
                     0, 0, 0, 1);

    // 绕 Y 轴旋转
    auto cs = cossin(vec3(1, 0, 0), vec3(b.x, 0, b.z));
    if (b.z < 0) cs.y = -cs.y;
    auto ry = mat4x4(cs.x, 0, cs.y, 0,
                     0, 1, 0, 0,
                     -cs.y, 0, cs.x, 0,
                     0, 0, 0, 1);
    auto ryr = ry.transpose;

    // 绕 Z 轴旋转
    cs = cossin(vec3(b.x, b.y, b.z), vec3(b.x, 0, b.z));
    if (b.y > 0) cs.y = -cs.y;
    auto rz = mat4x4(cs.x, -cs.y, 0, 0,
                     cs.y, cs.x, 0, 0,
                     0, 0, 1, 0,
                     0, 0, 0, 1);
    auto rzr = rz.transpose;

    // 绕 X 轴旋转
    auto rx = mat4x4(1, 0, 0, 0,
                     0, cos(angle), -sin(angle), 0,
                     0, sin(angle), cos(angle), 0,
                     0, 0, 0, 1);

    // 计算最终结果
    auto v = tr * ryr * rzr * rx * rz * ry * t * vec4(p, 1);
    return vec3(v.x, v.y, v.z);
}
```
