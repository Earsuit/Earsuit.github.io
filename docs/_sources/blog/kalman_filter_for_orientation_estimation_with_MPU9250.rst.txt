Kalman Filter for orientation estimation with MPU9250
#######################################################

.. post:: 07, Mar 2020
    :tags: Microcontroller, MPU9250
    :category: Microcontroller
    :author: Earsuit

Kalman Filter was proposed by Rudolph E.Kalman in 1960, it is the optimal estimator (linear systems) for a 
large class of problems. It produces estimates of unknown variables given a sequence of previous 
measurements, even if these measurements contain noise.

A brief intro to discrete Kalman Filter
========================================

Kalman Filter
----------------

Consider the following linear system and observation model with known initial optimal estimate :math:`\boldsymbol{x}_0^a` and error covariance :math:`\boldsymbol{P}_0 = E[(\boldsymbol{x}_0 - \boldsymbol{x}_0^a)(\boldsymbol{x}_0 - \boldsymbol{x}_0^a)^T]`:

.. math::
	:label: eq:con_state_kf

	\begin{equation} \boldsymbol{x}_k = \boldsymbol{A}\boldsymbol{x}_{k-1}+\boldsymbol{B}\boldsymbol{u}_{k-1}+\boldsymbol{w} \label{eqn:con_state_kf}\end{equation}

.. math::

	\begin{equation} \boldsymbol{z}_k = \boldsymbol{H}\boldsymbol{x}_{k}+\boldsymbol{v} \end{equation}

Where the input :math:`\boldsymbol{u}` is a know nonrandom vector. We assume that :math:`\boldsymbol{w}` 
captures the uncertainties in the system model and :math:`\boldsymbol{v}` denotes the measurement noise. 
The :math:`\boldsymbol{w}` and :math:`\boldsymbol{v}` are white noise with known covariances and are both 
uncorrelated with the initial state :math:`\boldsymbol{x}_0`.

Let's define :math:`E[\boldsymbol{w}\boldsymbol{w}^T] = \boldsymbol{Q},\ E[\boldsymbol{v}\boldsymbol{v}^T] = \boldsymbol{R}`.

The Kalman filter has two steps, first is to obtain "a priori" state through the system model, then correct 
that state with the measurement:

- Step 1:

.. math::

	\begin{equation}\boldsymbol{x}_k^f = \boldsymbol{A}\boldsymbol{x}_{k-1}^a + \boldsymbol{B}_{k-1}\boldsymbol{u}_{k-1}\end{equation}

.. math::

	\begin{equation}\boldsymbol{P}_k^f = \boldsymbol{A}\boldsymbol{P}_{k-1}\boldsymbol{A}_{k-1}^T + \boldsymbol{Q} \end{equation}

- Step 2:

.. math::

	\begin{equation}\boldsymbol{x}_k^a = \boldsymbol{x}_{k}^f + \boldsymbol{K}_{k}(\boldsymbol{z}_k-\boldsymbol{H}\boldsymbol{x}_k^f)\end{equation}

.. math::
	:label: eq:kalman_gain

	\begin{equation}\boldsymbol{K}_k = \boldsymbol{P}_k^f\boldsymbol{H}^T(\boldsymbol{H}\boldsymbol{P}_k^f\boldsymbol{H}^T + \boldsymbol{R})^{-1} \end{equation}

.. math::

	\begin{equation}\boldsymbol{P}_k = (\boldsymbol{I} - \boldsymbol{K}_k\boldsymbol{H})\boldsymbol{P}_k^f \end{equation}

Extended Kalman Filter
-----------------------

The original Kalman Filter can only deal with linear systems, but most systems in reality are nonlinear. 
Luckily, we can extend it to the nonlinear world with Taylor Series expansions.

Assume we have the following nonlinear system:

.. math::

	\begin{equation} \boldsymbol{x}_k = \boldsymbol{f}(\boldsymbol{x}_{k-1}) + \boldsymbol{w} \end{equation}

.. math::

	\begin{equation} \boldsymbol{z}_k = \boldsymbol{h}(\boldsymbol{x}_k) + \boldsymbol{v}\end{equation}

Where :math:`\boldsymbol{f(.)}` and :math:`\boldsymbol{h(.)}` are the nonlinear process function and 
nonlinear observation function respectively.

The initial state :math:`\boldsymbol{x}_0^a` and covariance :math:`\boldsymbol{P}_0` are known, and we have 
:math:`E[\boldsymbol{w}\boldsymbol{w}^T] = \boldsymbol{Q}\), \(E[\boldsymbol{v}\boldsymbol{v}^T] = \boldsymbol{R}`.

The Extended Kalman Filter also has two steps:

- Step 1:

.. math::
	:label: eq:step1_1

	\begin{equation}\boldsymbol{x}_k^f \approx \boldsymbol{f}(\boldsymbol{x}_{k-1}^a) \label{eqn:step1}\end{equation}

.. math::
	:label: eq:step1_2

	\begin{equation}\boldsymbol{P}_k^f = \boldsymbol{J_f}(\boldsymbol{x}_{k-1}^a)\boldsymbol{P}_{k-1}\boldsymbol{J_f}^T(\boldsymbol{x}_{k-1}^a)+ \boldsymbol{Q} \label{eqn:step2}\end{equation}

- Step 2:

.. math::

	\begin{equation} \boldsymbol{x}_k^a \approx \boldsymbol{x}_k^f + \boldsymbol{K}_k\left(\boldsymbol{z}_k - \boldsymbol{h}(\boldsymbol{x}_k^f)\right) \end{equation}

.. math::

	\begin{equation} \boldsymbol{K}_k = \boldsymbol{P}_k^f\boldsymbol{J_h}^T(\boldsymbol{x}_k^f) \left(\boldsymbol{J_h}(\boldsymbol{x}_k^f)\boldsymbol{P}_k^f\boldsymbol{J_h}^T(\boldsymbol{x}_k^f) + \boldsymbol{R}\right ) ^{-1} \label{eqn:kalman_gain}\end{equation}

.. math::

	\begin{equation} \boldsymbol{P}_k = \left(\boldsymbol{I} - \boldsymbol{K}_k\boldsymbol{J_h}(\boldsymbol{x}_k^f)\right) \boldsymbol{P}_k^f\end{equation}

Where

.. math::

	\boldsymbol{J_f}(\boldsymbol{x}) = \begin{bmatrix} \frac{\partial{f}_1}{\partial{x}_1} & \frac{\partial{f}_1}{\partial{x}_2} & \dots & \frac{\partial{f}_1}{\partial{x}_n}\\ \vdots & \vdots & \ddots & \vdots \\ \frac{\partial{f}_n}{\partial{x}_1} & \frac{\partial{f}_n}{\partial{x}_2} & \dots & \frac{\partial{f}_n}{\partial{x}_n}\end{bmatrix} \
	\boldsymbol{J_h}(\boldsymbol{x}) = \begin{bmatrix} \frac{\partial{h}_1}{\partial{x}_1} & \frac{\partial{h}_1}{\partial{x}_2} & \dots & \frac{\partial{h}_1}{\partial{x}_n}\\ \vdots & \vdots & \ddots & \vdots \\ \frac{\partial{h}_n}{\partial{x}_1} & \frac{\partial{h}_n}{\partial{x}_2} & \dots & \frac{\partial{h}_n}{\partial{x}_n}\end{bmatrix}


With these equations, we can apply it on nonlinear systems. However, there ain't no such thing as a free 
lunch. The extended Kalman filter is not an optimal estimator (it may reach a locally optimal), and it may 
diverge quickly if the initial state or the process model is incerrect.

A brief intro to quaternion
=============================

Here we will use quaternions to represent angular position instead of Euler angles as quaternion will not 
cause gimbal lock.

Let quaternion :math:`q` represent an angular position:

.. math::
	:label: eq:quaternion

	\begin{equation} \boldsymbol{q} = \begin{bmatrix} q_0 & q_1 & q_2 & q_3 \end{bmatrix}^T = q_0 + q_1\boldsymbol{i} + q_2\boldsymbol{j} + q_3\boldsymbol{k} \label{eqn:quaternion} \end{equation}

where :math:`\hat{\boldsymbol{i}},\hat{\boldsymbol{j}},\hat{\boldsymbol{k}}` satisify

.. math::

	\begin{equation*}\begin{aligned}\hat{\boldsymbol{i}}^2 & = \hat{\boldsymbol{j}}^2 = \hat{\boldsymbol{k}}^2 = \hat{\boldsymbol{i}}\hat{\boldsymbol{j}}\hat{\boldsymbol{k}} = -1 \\
	\hat{\boldsymbol{i}}\hat{\boldsymbol{j}} & = \hat{\boldsymbol{k}} = -\hat{\boldsymbol{j}}\hat{\boldsymbol{i}}\\
	\hat{\boldsymbol{j}}\hat{\boldsymbol{k}} & = \hat{\boldsymbol{i}} = -\hat{\boldsymbol{k}}\hat{\boldsymbol{j}}\\
	\hat{\boldsymbol{k}}\hat{\boldsymbol{i}} & = \hat{\boldsymbol{j}} = -\hat{\boldsymbol{i}}\hat{\boldsymbol{k}}\end{aligned}\end{equation*}

The addition of quaternions is component-wise, but this is not the case for multiplication. The product of 
two quaternion is given by:

.. math::

	\begin{equation*} \begin{aligned} \boldsymbol{q}_1\boldsymbol{q}_2 & = (a+b\hat{\boldsymbol{i}}+c\hat{\boldsymbol{j}} + d\hat{\boldsymbol{k}})(e+f\hat{\boldsymbol{i}}+g\hat{\boldsymbol{j}} + h\hat{\boldsymbol{k}})\\
	& = (ae-bf-cg-df) + \\
	& (be+af-dg+ch)\hat{\boldsymbol{i}} + \\
	& (ce+df+ag-bh)\hat{\boldsymbol{j}} + \\
	& (de-cf+bg+ah))\hat{\boldsymbol{k}}
	\end{aligned}\end{equation*}

This looks a bit messy, luckily, here is a more elegant way to represent quaternion multiplication which is 
called *Graßmann Product* if let :math:`\boldsymbol{u} = \begin{bmatrix}b\\c\\d \end{bmatrix}, \boldsymbol{v} = \begin{bmatrix}f\\g\\h \end{bmatrix}`:

.. math::

	\begin{equation} \boldsymbol{q}_1\boldsymbol{q}_2 \end{equation} = [ae - \boldsymbol{u}\cdot\boldsymbol{v},a\boldsymbol{v} + e\boldsymbol{u} + \boldsymbol{u}\times\boldsymbol{v}]

We can also express the multiplication in matrix form:

.. math::

	\begin{equation} \boldsymbol{q}_1\boldsymbol{q}_2 = \begin{bmatrix}a &-b&-c&-d\\b & a & -d & c\\c &d & a& -b \\d & -c& b & a\end{bmatrix}\begin{bmatrix}e\\f\\g\\h \end{bmatrix} \end{equation}

Equation :math:numref:`eq:quaternion` is the general form of a quaternion, we can write it in another form 
which exposes the axis of rotation and angle of rotation:

.. math::

	\boldsymbol{q} = (cos(\frac{\theta}{2}), sin(\frac{\theta}{2})\boldsymbol{u})

Where :math:`\boldsymbol{u} = \begin{bmatrix}x&y&z\end{bmatrix}` is the axis of rotation. One important thing 
is that the quaternion has to be a unit quaternion (:math:`|q| = \sqrt{q_0^2+q_1^2+q_2^2+q_3^2} = 1`) if it 
represents an angle of rotation, otherwise it will stretch the vector to be rotated.

The formula for rotating a vector :math:`\boldsymbol{v}` around axis :math:`\boldsymbol{u}` through angle 
:math:`\theta` is given by:

.. math::

	\begin{equation} \boldsymbol{v}' = \boldsymbol{q}\boldsymbol{v}\boldsymbol{q}^* \end{equation}

where :math:`\boldsymbol{q}^* = (cos(\frac{\theta}{2}), -sin(\frac{\theta}{2})\boldsymbol{u})` is the conjugate pair.

Like multiplication, it's also easy to write quaternion rotation in matrix format:

.. math::

	\boldsymbol{v}' = \boldsymbol{q}\boldsymbol{v}\boldsymbol{q}^* = \begin{bmatrix}1 - 2c^2-2d^2& 2bc-2ad& 2ac+2bd\\2bc+2ad&1-2b^2-2d^2&2cd-2ab\\2bd-2ac&2ab+2cd&1-2b^2-2c^2\end{bmatrix}\boldsymbol{v}

given :math:`\boldsymbol{q} = [a,b,c,d]`.

Let's take a look at quaternion differentiation. Assume at time :math:`t+\Delta t`, a rotation is described 
by :math:`q(t+\Delta t)`, i.e. :math:`\boldsymbol{q}(t) \rightarrow \boldsymbol{q}(t+\Delta t)` after time 
:math:`\Delta t`. The extra rotation performed during :math:`\Delta t` is around an instantaneous axis 
:math:`\hat{\boldsymbol{\omega}} = \boldsymbol{\omega}/|\boldsymbol{\omega}|` through the angle 
:math:`\Delta \theta = |\boldsymbol{\omega}|\Delta t`, where :math:`\boldsymbol{\omega}` is the angular 
velocity.

The change of quaternion is given by

.. math::

	\begin{equation*} \begin{aligned} \Delta \boldsymbol{q} & = cos(\frac{\Delta\theta}{2}) + \hat{\boldsymbol{\omega}}sin(\frac{\Delta\theta}{2}) \\
	& = cos(\frac{|\boldsymbol{\omega}|\Delta t}{2}) + \hat{\boldsymbol{\omega}}sin(\frac{|\boldsymbol{\omega}|\Delta t}{2})\end{aligned}\end{equation*}

Then :math:`\boldsymbol{q}(t+\Delta t) = \Delta \boldsymbol{q} \boldsymbol{q}(t)`. Let's first write down the difference:

.. math::

	\begin{equation*} \begin{aligned} \boldsymbol{q}(t+\Delta t) - \boldsymbol{q}(t) & = \left(cos(\frac{|\boldsymbol{\omega}|\Delta t}{2}) +\hat{\boldsymbol{\omega}}sin(\frac{|\boldsymbol{\omega}|\Delta t}{2})\right)\boldsymbol{q}(t) - \boldsymbol{q}(t) \\
	& = -2sin^2(\frac{|\boldsymbol{\omega}|\Delta t}{4})\boldsymbol{q}(t) + \hat{\boldsymbol{\omega}}sin(\frac{|\boldsymbol{\omega}|\Delta t}{2})\boldsymbol{q}(t)
	\end{aligned}\end{equation*}

With the definition of differentiation, we can have

.. math::

	\begin{equation*} \begin{aligned} \dot{\boldsymbol{q}}(t) & = \lim_{\Delta t \rightarrow 0}(\frac{\boldsymbol{q}(t+\Delta t) - \boldsymbol{q}(t)}{\Delta t})\\
	& = \lim_{\Delta t \rightarrow 0}\left(-\frac{2sin^2(\frac{|\boldsymbol{\omega}|\Delta t}{4})\boldsymbol{q}(t)}{\Delta t} + \hat{\boldsymbol{\omega}}\frac{sin(\frac{|\boldsymbol{\omega}|\Delta t}{2})\boldsymbol{q}(t)}{\Delta t} \right) \\
	& = -\lim_{\Delta t \rightarrow 0}\frac{2sin^2(\frac{|\boldsymbol{\omega}|\Delta t}{4})\boldsymbol{q}(t)}{\Delta t} +\lim_{\Delta t \rightarrow 0}\hat{\boldsymbol{\omega}}\frac{sin(\frac{|\boldsymbol{\omega}|\Delta t}{2})\boldsymbol{q}(t)}{\Delta t} \\
	& = -\lim_{\Delta t \rightarrow 0}\frac{|\boldsymbol{\omega}|sin(\frac{|\boldsymbol{\omega}|\Delta t}{4})cos(\frac{|\boldsymbol{\omega}|\Delta t}{4})}{1} + \lim_{\Delta t \rightarrow 0}\hat{\boldsymbol{\omega}}|\boldsymbol{\omega}|\frac{\frac{1}{2}cos(\frac{|\boldsymbol{\omega}|\Delta t}{2})\boldsymbol{q}(t)}{1} \\
	& = \hat{\boldsymbol{\omega}}|\boldsymbol{\omega}|\frac{1}{2}\boldsymbol{q}(t) \\
	& = \frac{1}{2}\boldsymbol{\omega}\boldsymbol{q}(t)
	\end{aligned}\end{equation*}

To compute :math:`\dot{q}(t)`, the vector :math:`\boldsymbol{\omega}` can be extended to a quaternion 
:math:`\Omega = [0, \boldsymbol{\omega}]`.

System modeling
=================

Let a quaternion :math:`\boldsymbol{q}` represent the relative orientation of a system with respect to the 
inertial frame, the initial orientation is :math:`\boldsymbol{q} = [1,0,0,0]^T`.

Assume the orientation at time :math:`t` is :math:`\boldsymbol{q}(t)`, and the angular velocity is 
:math:`\boldsymbol{\omega} = [\omega_x, \omega_y, \omega_z]`. We known that the continuous-time state 
equation for quaternion is given by

.. math::
	:label: eq:con_state

	\begin{equation} \dot{q}(t) = \frac{1}{2}\boldsymbol{\Omega}q(t) \label{eqn:con_state} \end{equation}

where

.. math::

	\boldsymbol{\Omega} = \begin{bmatrix}0&-\omega_x&-\omega_y & -\omega_z\\\omega_x & 0 & \omega_z & -\omega_y\\ \omega_y& -\omega_z & 0 & \omega_x \\\omega_z&\omega_y& -\omega_x & 0\end{bmatrix}

Comparing equation :math:numref:`eq:con_state` with equation :math:numref:`eq:con_state_kf`,there is no 
explicit external input :math:`\boldsymbol{u}`. All inputs are contained in the state matrix 
:math:`\frac{1}{2}\boldsymbol{\Omega}`, which can be treated as a time-variant state matrix.

The next step is to discretise it to fit the discrete Kalman Filter. Here we use the Euler method to 
approximately discretise it, however, the properties of the continues system only hold in the discretised 
system for small :math:`\Delta t`:

.. math::

	\begin{aligned}\dot{\boldsymbol{q}} = \lim_{\Delta t \rightarrow 0}\frac{\boldsymbol{q}(t+\Delta t) - \boldsymbol{q}(t)}{\Delta t} & = \frac{1}{2}\boldsymbol{\Omega}\boldsymbol{q}(t)\\
	\boldsymbol{q}(t+\Delta t) & = \boldsymbol{q}(t) + \frac{1}{2}\boldsymbol{\Omega}\boldsymbol{q}(t)\Delta t\\
	\boldsymbol{q}(t+\Delta t)& = (\boldsymbol{I} + \frac{1}{2}\boldsymbol{\Omega}\Delta t)\boldsymbol{q}(t)
	\end{aligned}

Let :math:`\boldsymbol{A} = (\boldsymbol{I} + \frac{1}{2}\boldsymbol{\Omega}\Delta t)` represet the discrete 
state matrix. The discrete time equation of state is given by:

.. math::

	\begin{equation} \boldsymbol{q}[k] = (\boldsymbol{I} + \frac{1}{2}\boldsymbol{\Omega}_k\Delta t) = \boldsymbol{A}_k\boldsymbol{q}[k-1] \label{eqn:dis_state_eqn}\end{equation}

with this equation of state, we could define two system outputs - the estimated gravity and magnetic ﬁeld in 
the body frame, which can be expressed by :math:`\boldsymbol{h}_1` and :math:`\boldsymbol{h}_2` respectively. 
The gravity and magnetic field in body frame after rotating from the inertial frame are given below:

.. math::

	\boldsymbol{h}_{1k} = \boldsymbol{R}_i^b \begin{bmatrix}0\\0\\g\end{bmatrix} = \begin{bmatrix}2q_1[k]q_3[k]-2q_0[k]q_2[k]\\2q_0[k]q_1[k]+2q_2[k]q_3[k]\\q_0^2[k]-q_1^2[k]-q_2^2[k]+q_3^2[k]\end{bmatrix}

.. math::

	\boldsymbol{h}_{2k} = \boldsymbol{R}_i^b \begin{bmatrix}0\\1\\0\end{bmatrix} = \begin{bmatrix}2q_1[k]q_2[k]+2q_0[k]q_3[k]\\q_0^2[k]-q_1^2[k]-q_2^2[k]-q_3^2[k]\\2q_2[k]q_3[k]-2q_0[k]q_1[k]\end{bmatrix}

where :math:`\boldsymbol{R}_i^b = \begin{bmatrix}1-2q_2^2-2q_3^2&2q_1q_2+2q_0q_3&2q_1q_3-2q_0q_2\\2q_1q_2-2q_0q_3&1-2q_1^2-2q_3^2&2q_2q_3+2q_0q_1\\2q_1q_3+2q_0q_2 & 2q_2q_3-2q_0q_1 & 1-2q_1^2-2q_2^2\end{bmatrix}` 
is the rotation matrix. The magnetic field in inertial frame 
:math:`\boldsymbol{B}_i = [0,1,0]^T`, normalise to one, is what we assume that the direction of the magnetic 
filed is along the y axis. You can also assume it has a different direction, like x axis, the consequence of 
these assumptions is that the initial yaw estimation from the Kalman Filter is usually none zero.

Filtering
==========

The orientation estimation algorithm is based on A Double-Stage Kalman Filter for Orientation Tracking With 
an Integrated Processor in 9-D IMU. For more details, please refer to this paper.

As the output equations are nonlinear, the extended Kalman Filter comes into play.

The code can be downloaded `here <https://github.com/Earsuit/kalman_filter_mpu9250>`_. Please note that the 
raw gyro, acceleration & magnetic field readings have 
to be calibrated before filtering. The calibration procedure can be found on one of my 
`blog <https://github.com/Earsuit/kalman_filter_mpu9250>`_. A recursive 
least-squares method is implemented to perform online ellipsoid fitting in the code.

Step 1
-------

Let's first go through the Step 1 in extended Kalman Filter. Compute the forcast position 
:math:`\boldsymbol{q}^-` through equation :math:numref:`eq:step1_1` with angular velocities measured from the 
gyroscope and the last best estimation :math:`\boldsymbol{q}^a`:

.. math::

	\begin{equation} \boldsymbol{q}_k^f = \boldsymbol{A}_{k-1}\boldsymbol{q}_{k-1}^a\end{equation}

Then compute the forecast noise covariance matrix through equation :math:numref:`eq:step1_2`. Luckily, the 
state equation is linear, gives:

.. math::

	\begin{equation}\boldsymbol{P}_k^f = \boldsymbol{A}_{k-1}\boldsymbol{P}_{k-1}\boldsymbol{A}_{k-1}^T + \boldsymbol{Q} \end{equation}

These are what we can do with the previous best estimation and system input.

Step 2
-------

With measurements, we could correct what we forecasted in the previous step.

Let's take a look at what the gravity measurement can bring to us.

.. figure:: ../assets/kalman_filter_for_orientation_estimation_with_MPU9250/roll-pitch-yaw.png

	Roll-Pitch-Yaw

Let :math:`x,y,z` axes align with roll(:math:`\alpha`), pitch(:math:`\beta`), yaw(:math:`\gamma`) 
respectively. The rotation matrix that rotates from inertial frame to body frame with rotation sequence z-x-y 
is given by:

.. math::

	\small\boldsymbol{R} = \begin{bmatrix}cos(\beta)cos(\gamma)-sin(\alpha)sin(\beta)sin(\gamma)& cos(\beta)sin(\gamma)+sin(\alpha)sin(\beta)cos(\gamma) & -cos(\alpha)sin(\beta) \\-cos(\alpha)sin(\gamma) & cos(\alpha)cos(\gamma) & sin(\alpha) \\ sin(\beta)cos(\gamma)+cos(\beta)sin(\alpha)sin(\gamma)&sin(\beta)sin(\gamma)-cos(\beta)sin(\gamma)cos(\gamma) & cops(\alpha)cos(\beta)\end{bmatrix}

The gravity in inertial frame is :math:`\boldsymbol{g}_i = [0,0,|g|]^T`, with :math:`\boldsymbol{R}` we can express it in body frame:

.. math::

	\boldsymbol{g}_b = \boldsymbol{R}\boldsymbol{g}_i = |g|\begin{bmatrix}-cos(\alpha)sin(\beta)\\sin(\alpha)\\cos(\alpha)cos(\beta) \end{bmatrix}

It shows that the we can only compute roll and pitch from gravity measurement as there is no \(\gamma\) in 
the above equation. That's why we need a 9-axis IMU sensor as the yaw can be obtained through computing the 
angle between two consecutive magnetic filed readings in inertial frame.

As two measurements are needed to correct the forecasted state, the correction is composed of two stages, 
first correct the roll and pitch with gravity measurement, then yaw with magnetometer reading.

.. _stage_1:

Stage 1
^^^^^^^^

According to equation :math:numref:`eq:kalman_gain`, the Jacobian matrix 
:math:`\boldsymbol{J}_{\boldsymbol{y}_1k}` is required to compute the Kalman gain:

.. math::
	
	\boldsymbol{J}_{\boldsymbol{h}_1k} = \frac{\partial{\boldsymbol{h}_{1k}}}{\partial{\boldsymbol{q}_k}} = \begin{bmatrix} -2q_0 & 2q_3 & -2q_0 & 2q_1\\ 2q_1 & 2q_0 & 2q_3 & 2q_2 \\ 2q_0 & -2q_1 & -2q_2 & 2q_3 \end{bmatrix}

.. math::

	\boldsymbol{K}_{1k} = \boldsymbol{P}_k^f\boldsymbol{J}_{\boldsymbol{h}_1k}(\boldsymbol{J}_{\boldsymbol{h}_1k}\boldsymbol{P}_k^f\boldsymbol{J}_{\boldsymbol{h}_1k}^T + \boldsymbol{R})^{-1}

Then compute the correction factor 
:math:`\boldsymbol{q}_{\epsilon1k} = \boldsymbol{K}_{1k}(\boldsymbol{z}_{1k} - \boldsymbol{h}_{1k})`, where 
:math:`\boldsymbol{z}_{1k}` is the accelerometer reading. Because we don't want the gravity measurement 
affect the yaw estimation, the fourth element of :math:`\boldsymbol{q}_{\epsilon1k}` needs to be set to zero, 
which means the rotation axis is :math:`\boldsymbol{u}=[x,y,0]^T`.

The post error covariance matrix and corrected state are

.. math::

	\boldsymbol{P}_{1k} = (\boldsymbol{I} - \boldsymbol{K}_{1k}\boldsymbol{J}_{\boldsymbol{h}_1k})\boldsymbol{P}_k^f

.. math::

	\boldsymbol{q}_{1k}^a = \boldsymbol{q}_k^f + \boldsymbol{q}_{\epsilon1k}

Stage 2
^^^^^^^^

Stage 2 utilises the magnetometer reading to correct the :math:`\boldsymbol{q}_{1k}^a` obtained in :ref:`stage_1`. First compute the Jacobian matrix and Kalman gain:

.. math::

	\boldsymbol{J}_{\boldsymbol{h}_2k} = \frac{\partial{\boldsymbol{h}_{2k}}}{\partial{\boldsymbol{q}_k}} = \begin{bmatrix} 2q_3 & 2q_2 & 2q_1 & 2q_0\\ 2q_0 & -2q_1 & -2q_2 & -2q_3 \\ -2q_1 & -2q_0 & 2q_3 & 2q_2 \end{bmatrix}

.. math::

	\boldsymbol{K}_{2k} = \boldsymbol{P}_k^f\boldsymbol{J}_{\boldsymbol{h}_2k}(\boldsymbol{J}_{\boldsymbol{h}_2k}\boldsymbol{P}_k^f\boldsymbol{J}_{\boldsymbol{h}_2k}^T + \boldsymbol{R})^{-1}

Then compute the correction factor with magnetometer reading :math:`\boldsymbol{z}_{2k}`, 
:math:`\boldsymbol{q}_{\epsilon2k} = \boldsymbol{K}_{2k}(\boldsymbol{z}_{2k} - \boldsymbol{h}_{2k})`, and set 
the second and third element to zero to eliminate the effect on roll and pitch.

Finally update the post error covariance and corrected state:

.. math::

	\boldsymbol{P}_{2k} = (\boldsymbol{I} - \boldsymbol{K}_{2k}\boldsymbol{J}_{\boldsymbol{h}_2k})\boldsymbol{P}_{1k}

.. math::

	\boldsymbol{q}_{k}^a = \boldsymbol{q}_{1k}^a + \boldsymbol{q}_{\epsilon2k}