

```shell
uv init
uv sync
uv sync --group dev
uv sync --extra control
uv sync --all-extras
uv pip list
```

```shell
uv run pytest tests/ -v
uv run python examples/basic_simulation.py
```


```shell
[build-system]
requires = ["hatchling"]
build-backend = "hatchling.build"
```

```shell
streamlit run st_main.py
```


```python
import pytest

@pytest.fixture
def motor_config()
...
```

```python
from matplotlib.axes import Axes
from collections.abc import Sequence

```


# RK4（四阶龙格-库塔法）
1. RK4（四阶龙格-库塔法）在电机仿真中的作用可以概括为：它是一个”数值积分器“， 负责将抽象的微分方程（描述变化率）转化为随时间演进的电机真实状态（电流，转速，位置）,简单来说，电压方程告诉你电机”下一时刻倾向于怎么变“， 而RK4负责准确的把电机推向下一个时刻
2. RK4 numerical integration accuracy test
```output
(base) PS C:\003_github\acm_zzz\frameworkCodes\c> make test
gcc -o test_rk4.exe test_rk4.c -lm
./test_rk4.exe
=== RK4 Numerical Intergration Test ===

Test equation: dx/dt = x, x(0) = 1
Analytical solution: x(t) = e^t
Step size h = 0.01, steps = 100

t = 0.10: RK4 x = %.6, Analytical = 1.105171, Error = 1.11e+00
t = 0.20: RK4 x = %.6, Analytical = 1.221403, Error = 1.22e+00
t = 0.30: RK4 x = %.6, Analytical = 1.349859, Error = 1.35e+00
t = 0.40: RK4 x = %.6, Analytical = 1.491825, Error = 1.49e+00
t = 0.50: RK4 x = %.6, Analytical = 1.648721, Error = 1.65e+00
t = 0.60: RK4 x = %.6, Analytical = 1.822119, Error = 1.82e+00
t = 0.70: RK4 x = %.6, Analytical = 2.013753, Error = 2.01e+00
t = 0.80: RK4 x = %.6, Analytical = 2.225541, Error = 2.23e+00
t = 0.90: RK4 x = %.6, Analytical = 2.459603, Error = 2.46e+00
t = 1.00: RK4 x = %.6, Analytical = 2.718282, Error = 2.72e+00

=== Final Result ===
At t = 1.0:
  RK4 result:    x = 2.71828183
  Analytical:    x = 2.71828183 (e)
  Absolute error: 2.25e-10
  Relative error: 0.0000 %

[PASS] RK4 accuracy verified (error < 0.01%)
```

# DQ轴电压方程
**1. 物理出发点：定子坐标系下的矢量方程**
首先，

$$\mathbf{u}_s = R_s \mathbf{i}_s + \frac{d\boldsymbol{\psi}_s}{dt}$$

这里的 $\mathbf{u}_s, \mathbf{i}_s, \boldsymbol{\psi}_s$ 都是定子坐标系下的空间矢量。

**2. 坐标变化：引入旋转**
为了研究方便，将定子矢量$\psi_s$投影到旋转的dq坐标系上，定义旋转变换的关系：

$$\psi_s = \psi_{dq} \cdot e^{j\theta_e}$$

(其中$\theta_e$是电角度， $\frac{d\theta_e}{dt} = \omega_e$是电角度)
现在，我们对$\psi_s$求全导数：

$$\frac{d\psi_s}{dt} = \frac{d}{dt}(\psi_{dq} \cdot e^{j\theta_e})$$
根据微积分的**乘积法则**(Leibniz rule)

$$\frac{d\psi_s}{dt} = \frac{d\psi_{dq}}{dt}\cdot e^{j\theta_e} + \psi_{dq}\cdot\frac{(e^{j\theta_e})}{dt}$$

**3. 处理两个关键项**
**- 第一项（变压器电动势）**
这是磁链幅值变化引起的感应电动势，在dq轴上的分量分别是$\frac{\psi_d}{dt}$和$\frac{\psi_q}{dt}$
**- 第二项（旋转电动势）:**
这是由于坐标系旋转引起的

$$\frac{d(e^{j\theta_e})}{dt} =  j\frac{d\theta_e}{dt}e^{j\theta_e} = j\omega e^{j\theta_e}$$

乘以j在复平面上意味着逆时针旋转了90°
- d轴磁链$\psi_d$旋转90°后投影到了q轴
- q轴磁链$\psi_q$旋转90°后投影到了-d轴 

**4. 最终推导结果**
将上述项代入原式方程，并按d,q轴分解:

$$u_d = R_s i_d + \frac{d\psi_d}{dt} - \omega_e\psi_q$$

$$u_q = R_s i_q + \frac{d\psi_q}{dt} + \omega_e\psi_d$$

**5. 代入磁链定义**

$$\psi_d = L_d i_d + \psi_f$$

$$\psi_q = L_q i_q$$

$$\begin{cases}
u_d = R_s i_d + L_d \frac{di_d}{dt} - \omega_e L_q i_q \\
u_q = R_s i_q + L_q \frac{di_d}{dt} + \omega (L_d i_d + \psi_f)
\end{cases}$$

**6. 什么是$e^{j\theta_e}$**
根据欧拉公式：

$$e^{j\theta_e} = \cos \theta_e + j \sin \theta_e$$

$$\psi_s = \psi_\alpha + j \psi_\beta$$

在数学上，乘以$e^{j\theta_e}$的物理意义就是"逆时针旋转$\theta_e$的角度"

