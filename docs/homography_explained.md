# Homography, Affine, Planar — Explained From Scratch (for the `h_vs` controller)

**Who this is for:** you've taken linear algebra — vectors, matrices, matrix
multiplication, inverse, transpose, the identity matrix. That's all you need.
No projective geometry, no Lie groups, no differential geometry. We build
everything else from those pieces and then read the actual `h_vs` code with it.

---

## 1. The one-sentence idea

> A **homography** is a single $3\times3$ matrix that describes how a **flat
> surface** (a plane — e.g. the face of a stop sign) appears to move/warp
> between two camera views.

If you know where the four corners of the sign are *now*, and where you *want*
them to be, the homography is the matrix that connects those two pictures. The
`h_vs` controller measures that matrix and turns "how far is the matrix from
doing nothing" into "how should the drone move."

That's the whole story. The rest is unpacking three words: **homogeneous**,
**affine**, **planar**.

---

## 2. The "add a 1" trick: homogeneous coordinates

A pixel in an image is just a 2D point $(u, v)$. We're going to staple a `1` on
the end and write it as a 3D vector:

$$(u, v) \;\longrightarrow\; \begin{bmatrix} u \\ v \\ 1 \end{bmatrix}$$

Why bother? Two reasons, both pure linear algebra:

**Reason A — translation becomes multiplication.** Normally "shift right by
$t_x$, down by $t_y$" is an *addition*, not a matrix multiply. But with the
extra 1:

$$\begin{bmatrix} 1 & 0 & t_x \\ 0 & 1 & t_y \\ 0 & 0 & 1 \end{bmatrix}
\begin{bmatrix} u \\ v \\ 1 \end{bmatrix}
= \begin{bmatrix} u + t_x \\ v + t_y \\ 1 \end{bmatrix}$$

Now *every* geometric operation — rotate, scale, shift, even perspective — is a
single $3\times3$ matrix. That uniformity is the entire point.

**Reason B — we're allowed to rescale the whole vector.** In this homogeneous
world, these all mean the *same* pixel:

$$\begin{bmatrix} u \\ v \\ 1 \end{bmatrix}
\;\sim\;
\begin{bmatrix} 2u \\ 2v \\ 2 \end{bmatrix}
\;\sim\;
\begin{bmatrix} 10u \\ 10v \\ 10 \end{bmatrix}$$

The rule to get back to a real pixel: **divide by the last entry.** So
$(2u, 2v, 2) \to (2u/2, 2v/2) = (u,v)$. This "divide by the last number" is
exactly what produces *perspective* (far-away things look smaller), as we'll
see in a moment. Because of this rescaling freedom, a homography matrix only
matters **up to scale** — multiplying the whole $3\times3$ by 7 gives the same
transform. (That's why people "normalize" it so the bottom-right entry is 1.)

---

## 3. The ladder of transformations (rigid → projective)

All of these are $3\times3$ matrices acting on $(u,v,1)$. They differ only in
how much freedom the matrix has. Walk up the ladder:

| Name | Matrix shape | What it can do | What it preserves |
|---|---|---|---|
| **Translation** | $\begin{bmatrix} 1&0&t_x\\0&1&t_y\\0&0&1\end{bmatrix}$ | slide | everything but position |
| **Euclidean** (rigid) | rotation $R$ + translation, bottom row $[0\,0\,1]$ | rotate + slide | lengths, angles |
| **Similarity** | add a uniform scale $s$ | + zoom | angles, ratios |
| **Affine** | $\begin{bmatrix} a&b&t_x\\c&d&t_y\\0&0&1\end{bmatrix}$ | + shear, + non-uniform scale | **parallel lines stay parallel** |
| **Projective (homography)** | $\begin{bmatrix} a&b&t_x\\c&d&t_y\\ \mathbf{g}&\mathbf{h}&1\end{bmatrix}$ | + perspective | only **straight lines stay straight** |

The single most important row in that table is the **bottom row**.

- **Affine** has bottom row $[0,\,0,\,1]$. When you multiply, the last entry of
  the output stays exactly `1`, so the "divide by last entry" step does
  nothing. **No perspective.** Parallel lines in, parallel lines out. Think of
  a scanner or a photocopier: a flat page, viewed straight-on, scaled/sheared
  but never "tilted away."

- **Projective / homography** has a general bottom row $[g,\,h,\,1]$. Now the
  last entry of the output is $g\,u + h\,v + 1$, which **changes from pixel to
  pixel**. When you divide by it, points where that denominator is large get
  squished toward the center — that's perspective foreshortening. This is what
  lets railroad tracks converge to a vanishing point.

**So "affine" is just "a homography that forgot how to do perspective."** It is
the special case where $g = h = 0$. Keep this sentence; it is the key to the
whole `h_vs` story in Section 8.

### Tiny numeric example of the perspective divide
Take the projective matrix with $g = 0.001$, $h = 0$:
$$\begin{bmatrix} 1&0&0\\0&1&0\\0.001&0&1\end{bmatrix}
\begin{bmatrix} 500\\ 200\\ 1\end{bmatrix}
= \begin{bmatrix} 500\\ 200\\ 1.5\end{bmatrix}
\xrightarrow{\div 1.5}
\begin{bmatrix} 333\\ 133 \end{bmatrix}$$
The point got pulled inward, and *how much* depended on its $u$-coordinate
(500). An affine matrix ($g=h=0$) could never do that — it would leave the
last entry at 1 and pull nothing.

---

## 4. What "planar" means and why it's required

A homography is only *exactly* correct for points that all lie on **one flat
plane** in the real world. The face of a stop sign is a plane. A wall is a
plane. The ground is a plane.

The theorem (you don't need its proof): **two images of the same flat plane are
always related by a single $3\times3$ homography.** Move the camera however you
like — the four corners of that planar sign in view A map to the four corners in
view B through one matrix $H$. Four corner-pairs is exactly enough to solve for
$H$ (each corner gives 2 equations, $H$ has 8 free numbers, $4\times2 = 8$).

That's why the controller uses the **four bounding-box corners** of the
detected sign: a box is planar, and 4 corners pin down the homography exactly.

---

## 5. The camera matrix `K` (pixels ↔ rays)

A camera turns a 3D direction into a pixel. That conversion is a matrix `K`
(the *intrinsics*):

$$K = \begin{bmatrix} f_x & 0 & c_{x0} \\ 0 & f_y & c_{y0} \\ 0 & 0 & 1 \end{bmatrix}$$

- $f_x, f_y$ — focal lengths in pixels (how "zoomed in" the camera is).
- $(c_{x0}, c_{y0})$ — the **principal point**, i.e. the pixel at the optical
  center (essentially the middle of the image).

`K` maps a normalized 3D ray → a pixel. So $K^{-1}$ maps a **pixel → a
normalized ray** (a direction in the real world, stripped of the camera's
zoom/offset). Remember this: **$K^{-1}$ converts "where it is on screen" into
"which real-world direction it is."** We need that to talk about drone motion
in meters/radians instead of pixels.

---

## 6. The control law, exactly as the code computes it

Now we read [src/h_vs_servo/src/homography_2d_vs.cpp](../src/h_vs_servo/src/homography_2d_vs.cpp).
Here is the whole math core:

```cpp
Eigen::Matrix3d H = _K.inverse() * G * _K;          // line 15
Eigen::Vector3d m_star = _K.inverse() * p_star;     // line 16
twist << _lambda_v.asDiagonal() * _computeEv(H, m_star),   // (H - I) m*
         _lambda_w.asDiagonal() * _computeEw(H);           // vex(H - H^T)
```

Step by step:

### 6a. Measure `G` (pixel-space homography)
In [hvs_controller.cpp](../src/h_vs_servo/src/hvs_controller.cpp) the controller
builds two sets of four corners:
- `pts_star_` — where we **want** the box (centered, at the target size).
- `pts_curr` — where the box **is right now**.

Then OpenCV solves the 4-corner system for us:
```cpp
cv::Mat G_cv = cv::getPerspectiveTransform(pts_star_, pts_curr);  // line 72
```
`G` is the $3\times3$ that maps *desired pixels → current pixels*.

### 6b. Convert to the "Euclidean" homography `H`
$$H = K^{-1}\, G \, K$$
This is a *change of basis* (pure linear algebra: sandwich a matrix between
$K^{-1}$ and $K$). It re-expresses the same warp, but now in normalized ray
coordinates (meters/radians world) instead of pixels. Now $H$ has physical
meaning: it's how the camera is displaced from the goal.

**The key intuition:** if the drone is *exactly* where it should be, current =
desired, so $G$ does nothing, and $H = I$ (the identity matrix). **The error is
"how far is $H$ from the identity."** Everything below is just two different
ways of measuring "$H$ minus doing-nothing."

### 6c. Translation error — how far off in *position*
$$\mathbf{e}_v = (H - I)\, \mathbf{m}^* \qquad\text{(paper's eq. 15)}$$
`m*` is the desired reference ray. The code defaults it to the **principal
point**, $p^* = (c_{x0}, c_{y0}, 1)$, so $\mathbf{m}^* = K^{-1} p^* = (0,0,1)$ —
i.e. "straight ahead." Multiplying $(H-I)$ by $(0,0,1)$ just **picks out the
third column of $H-I$**:
$$\mathbf{e}_v = \begin{bmatrix} H_{0,2} \\ H_{1,2} \\ H_{2,2}-1 \end{bmatrix}.$$
These three numbers say: *how far the sign's center is off to the side*
($H_{0,2}$), *up/down* ($H_{1,2}$), and *in depth* ($H_{2,2}-1$).

### 6d. Rotation error — how far off in *orientation*
$$\mathbf{e}_\omega = \mathrm{vex}(H - H^\top) \qquad\text{(paper's eq. 16)}$$

This needs one fact you can prove with transpose rules. For **any** matrix $H$,
the matrix $S = H - H^\top$ is **skew-symmetric**, meaning $S^\top = -S$:
$$ (H - H^\top)^\top = H^\top - H = -(H - H^\top). \checkmark $$

A skew-symmetric $3\times3$ always looks like this — zeros on the diagonal,
and only **three** independent numbers:
$$S = \begin{bmatrix} 0 & -z & y \\ z & 0 & -x \\ -y & x & 0 \end{bmatrix}.$$

The operator $\mathrm{vex}(\cdot)$ ("vee") just **reads those three numbers out
into a vector** $(x, y, z)$ — it's the inverse of building the skew matrix. In
the code:
```cpp
return Eigen::Vector3d(H_skew(2,1), H_skew(0,2), H_skew(1,0));   // line 39
```
Those three numbers are the small rotation (roll/pitch/yaw of the camera)
needed to line back up with the sign. (Why does the antisymmetric part encode
rotation? Because a tiny rotation matrix is $I + S$ with $S$ skew-symmetric;
subtracting $H^\top$ cancels the symmetric stretch part and leaves the
rotation.)

### 6e. Stack into a 6-DOF twist, with gains
$$\text{twist} = \begin{bmatrix} \boldsymbol{\lambda}_v \odot \mathbf{e}_v \\
\boldsymbol{\lambda}_\omega \odot \mathbf{e}_\omega \end{bmatrix}
= [\,t_x, t_y, t_z,\; r_x, r_y, r_z\,]$$
$\odot$ is element-wise multiply (the gains scale each axis). The gains live in
[config/hil/bench_h_vs.yaml](../config/hil/bench_h_vs.yaml):
`lambda_v: [1.0, 1.0, 0.0]`, `lambda_w: [0.0, 0.4, 0.0]`.

---

## 7. From camera twist to drone motion (the body mapping)

[hvs_controller.cpp](../src/h_vs_servo/src/hvs_controller.cpp) lines 112–117
convert the camera twist into body velocities, after a **5-tick moving
average** (the `twist_buffer_` — averages the last 5 computations to kill
jitter; this is exactly why `h_vs` has the smoothest commands in the
benchmark):

```cpp
v.vx = std::max(0.0, k_fwd_ * (target_bbox_ratio_ - in.bbox_ratio)); // forward
v.vy = -avg(0);   // camera "off to the side" -> body left/right
v.vz = -avg(1);   // camera "up/down"          -> body up/down
v.wz = -avg(4);   // camera yaw (e_w[1])       -> body yaw
```

Note `vx` does **not** come from the homography — it's a plain proportional law
on bounding-box size (`bbox_ratio` is the box's height fraction, a stand-in for
range). The next section explains *why it has to be that way here*.

---

## 8. Why the simulator's box breaks the homography (the "affine trap")

This is the crux, and it's the issue called out in the paper and the report.

In the benchmark, detection comes from an **oracle** that draws a **perfectly
axis-aligned** rectangle (sides parallel to the image edges, never tilted). Map
four axis-aligned corners to four axis-aligned corners and `getPerspectiveTransform`
can satisfy it **without any perspective** — the solution is **affine**. From
Section 3, affine means the bottom row is $[0, 0, 1]$, so after $H = K^{-1}GK$:

$$H_{2,2} = 1 \quad\text{always.}$$

Two consequences fall straight out of the formulas in Section 6:

**(i) No forward motion.** The depth component of the translation error was
$\mathbf{e}_v[2] = H_{2,2} - 1 = 0$, *always*, regardless of how far the drone
is. The homography simply cannot sense or command range here. That's why the
code bypasses it with the proportional `vx = k_fwd·(target_ratio − ratio)`
(line 113). With a real, textured, tilted target the box corners would *not* be
axis-aligned, $G$ would be genuinely projective, $H_{2,2}\neq1$, and the
homography would produce forward motion on its own.

**(ii) "Lateral parking."** For an affine $H$, work out the two horizontal
error terms:
- translation sideways: $\mathbf{e}_v[0] = H_{0,2} = (c_x - c_{x0})/f_x$
- yaw rotation: $\mathbf{e}_\omega[1] = H_{0,2} - H_{2,0} = (c_x - c_{x0})/f_x - 0$

They are the **same number** — the horizontal pixel offset of the sign. So the
*same* error drives both "slide sideways" (`vy`) and "rotate" (`wz`). Rotating
is the cheaper way to make the sign appear centered, so the drone yaws to face
the sign from wherever it is — and the moment the sign looks centered, *both*
corrections shut off together. Result: it parks slightly **to the side** of the
sign, facing it diagonally, instead of squaring up in front. This is a
degeneracy of the *input* (a flat, untextured, axis-aligned box carries no
perspective info to tell "I'm off to the side" apart from "I'm rotated"), **not
a bug in the code.**

---

## 9. The paper's LaTeX symbols, decoded

From `paper.tex`, Section `\subsubsection{Controller architectures}`:

| LaTeX | Reads as | Plain meaning |
|---|---|---|
| `\mathbf{G}` | $\mathbf{G}$ | pixel-space homography from `getPerspectiveTransform` (desired→current corners) |
| `\mathbf{K}` | $\mathbf{K}$ | camera intrinsics (focal lengths + principal point) |
| `\mathbf{H}=\mathbf{K}^{-1}\mathbf{G}\mathbf{K}` | $H = K^{-1}GK$ | same warp re-expressed in real-world ray coords; $=I$ when on-target |
| `\mathbf{m}^{*}` | $\mathbf{m}^*$ | desired reference ray $K^{-1}p^*$; here $(0,0,1)$ = straight ahead |
| `\mathbf{e}_v=(\mathbf{H}-\mathbf{I})\,\mathbf{m}^{*}` | $\mathbf{e}_v=(H-I)\mathbf{m}^*$ | translation (position) error = 3rd column of $H-I$ |
| `\mathbf{e}_\omega=\mathrm{vex}(\mathbf{H}-\mathbf{H}^{\top})$ | $\mathbf{e}_\omega=\mathrm{vex}(H-H^\top)$ | rotation error = the 3 numbers of the skew part of $H$ |
| `\mathbf{H}_{2,2}=1\Rightarrow e_{v,z}=0` | $H_{2,2}=1\Rightarrow e_{v,z}=0$ | affine box ⇒ depth error is always zero ⇒ no forward velocity |
| `(c_x-c_{x0})/f_x` | $(c_x-c_{x0})/f_x$ | horizontal pixel offset of the sign (drives both `vy` and `wz`) |

When the paper says the homography law is *"image-based"* and *"places it
between pure image-based and pose-based control"*: it only ever measures things
in the image (the four corners), but because $H = K^{-1}GK$ secretly contains
rotation + scaled-translation structure, it borrows a little of the
"reconstruct the pose" flavor — hence "between."

---

## 10. File map (where each piece lives)

| Concept | File | Lines |
|---|---|---|
| Solve $G$ from 4 corners; build desired corners | [src/h_vs_servo/src/hvs_controller.cpp](../src/h_vs_servo/src/hvs_controller.cpp) | 43–72 |
| $H=K^{-1}GK$, $\mathbf{e}_v$, $\mathbf{e}_\omega$, twist | [src/h_vs_servo/src/homography_2d_vs.cpp](../src/h_vs_servo/src/homography_2d_vs.cpp) | 12–40 |
| `vex` (read 3 numbers from skew matrix) | [homography_2d_vs.cpp](../src/h_vs_servo/src/homography_2d_vs.cpp) | 37–40 |
| Camera-twist → drone body velocities; 5-tick average | [hvs_controller.cpp](../src/h_vs_servo/src/hvs_controller.cpp) | 87–117 |
| Gains $\boldsymbol{\lambda}_v,\boldsymbol{\lambda}_\omega$, `k_fwd`, buffer length | [config/hil/bench_h_vs.yaml](../config/hil/bench_h_vs.yaml) | 1–19 |
| Paper write-up + the affine-degeneracy caveats | [paper.tex](../paper.tex) | `sec:vs_arch`, `sec:vs_discuss` |

---

## 11. One-paragraph recap

Staple a `1` onto pixels so every transform is a $3\times3$ matrix. A
**homography** is the matrix relating two views of a **flat plane**; an
**affine** transform is the crippled homography whose bottom row is $[0,0,1]$,
so it can't do perspective. `h_vs` measures the homography $G$ from the four box
corners, rewrites it as $H=K^{-1}GK$ (which equals the identity when the drone
is on-target), and reads the *position* error off the third column of $H-I$ and
the *orientation* error off the skew-symmetric part $H-H^\top$. Because the
simulator's boxes are axis-aligned, the measured homography collapses to affine,
which zeroes the depth term (no forward push from the homography → handled by a
separate proportional law) and makes the sideways and yaw errors identical
(→ the controller parks beside the sign). With a real, textured, tilted target
the full perspective returns and those degeneracies disappear.
