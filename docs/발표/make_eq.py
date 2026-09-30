"""발표용 수식을 진짜 LaTeX(Computer Modern)로 렌더링해 투명 PNG 로 만든다."""
import json
import os
import matplotlib

# 저장소 맨 위에서 실행한다: python3 docs/발표/make_eq.py  (TeX Live 필요)
os.environ["PATH"] = "/Library/TeX/texbin:" + os.environ["PATH"]
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams.update({"text.usetex": True,
                     "text.latex.preamble": r"\usepackage{amsmath}\usepackage{amssymb}\usepackage{bm}"})

EQ = {
    # 위치 추정
    "odom": r"$\Delta s=\dfrac{r\,(\Delta\phi_L+\Delta\phi_R)}{2},\qquad \theta_k=\operatorname{atan2}(m_x,\,m_y),\qquad \mathbf{x}_k=\mathbf{x}_{k-1}+\Delta s\begin{pmatrix}\cos\bar\theta\\ \sin\bar\theta\end{pmatrix}$",
    "slip": r"$\omega_{\mathrm{wheel}}=\dfrac{\Delta s_R-\Delta s_L}{B\,\Delta t},\qquad \bigl|\,\omega_{\mathrm{wheel}}-\dot\theta_{\mathrm{compass}}\bigr|>0.6\ \mathrm{rad/s}\ \ (0.3\,\mathrm{s})\ \Rightarrow\ \mathrm{slip}$",
    # 스캔 매칭
    "scan": r"$\boldsymbol{\delta}^{*}=\arg\min_{\boldsymbol{\delta}}\sum_{i} D(\mathbf{p}_i+\boldsymbol{\delta})^{2},\qquad (J^{\top}J)\,\Delta\boldsymbol{\delta}=-J^{\top}\mathbf{d},\quad J_i=\nabla D(\mathbf{p}_i+\boldsymbol{\delta})$",
    # 지도
    "logodds": r"$l=\log\dfrac{p}{1-p},\qquad l_t=l_{t-1}+\begin{cases}+0.9 & \text{hit}\\ -0.4 & \text{pass}\end{cases},\qquad l\in[-3,\,5]$",
    # 사과
    "range": r"$d_{\mathrm{size}}=\dfrac{R}{\sin\gamma},\qquad d_{\mathrm{ground}}=\dfrac{h}{\tan\delta},\qquad |d_{\mathrm{size}}-d_{\mathrm{ground}}|\le 0.3\,d_{\mathrm{ground}}$",
    # 칼만
    "kf": r"$\hat{\mathbf{x}}^-=F\hat{\mathbf{x}},\qquad P^-=FPF^{\top}+Q$",
    "kf2": r"$K=P^-H^{\top}(HP^-H^{\top}+R)^{-1},\qquad \hat{\mathbf{x}}=\hat{\mathbf{x}}^-+K(\mathbf{z}-H\hat{\mathbf{x}}^-)$",
    # 계획
    "astar": r"$f(n)=g(n)+h(n),\qquad h=\max(|\Delta r|,|\Delta c|)+(\sqrt{2}-1)\min(|\Delta r|,|\Delta c|)$",
    "inflate": r"$\mathrm{blocked}(c)\iff d(c,\mathrm{wall})\le r_{\mathrm{robot}}+m=0.13+0.22\ \mathrm{m}$",
    # 탐험
    "frontier": r"$k^{*}=\arg\min_{k}\ L_{A^{*}}(\mathbf{x}_{\mathrm{robot}},\,\mathbf{s}_k),\qquad \mathbf{s}_k:\ \text{standing spot of frontier }F_k$",
    # DWA
    "dwa": r"$J(v,\omega)=1.0\,\mathcal{N}\!\left(-\lVert\mathbf{p}_{T}-\mathbf{g}\rVert\right)+0.8\,\mathcal{N}\!\left(\min(c,\,0.8)\right),\qquad c>r_{\mathrm{robot}}+0.15$",
    "ik": r"$\omega_{L,R}=\dfrac{v\mp\omega B/2}{r},\qquad s=\min\!\left(1,\ \dfrac{\omega_{\max}}{\max(|\omega_L|,|\omega_R|)}\right),\qquad (\omega_L,\omega_R)\leftarrow s\,(\omega_L,\omega_R)$",
    # 시간 예산
    "budget": r"$t_{\mathrm{return}}=T-\max\!\left(1.5\,\dfrac{L_{\mathrm{home}}}{\bar v},\ 30\ \mathrm{s}\right)$",
}

sizes = {}
for name, tex in EQ.items():
    fig = plt.figure(figsize=(0.01, 0.01))
    fig.text(0, 0, tex, fontsize=20, color="#1B2631")
    path = f"docs/발표/assets/eq/{name}.png"
    fig.savefig(path, dpi=300, transparent=True, bbox_inches="tight", pad_inches=0.04)
    plt.close(fig)
    from PIL import Image
    w, h = Image.open(path).size
    sizes[name] = [w, h]
    print(name, w, h)
json.dump(sizes, open("docs/발표/assets/eq_sizes.json", "w"))
