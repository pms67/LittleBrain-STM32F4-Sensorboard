#include "EKF.h"

void EKF_Init(EKF *ekf, float P[2], float Q[2], float R[3]) {

	/* Reset state estimates */
	ekf->phi_r 		= 0.0f;
	ekf->theta_r 	= 0.0f;

	/* Initialise state covariance matrix */
	ekf->P[0][0] = P[0]; ekf->P[0][1] = 0.0f;
	ekf->P[1][0] = 0.0f; ekf->P[1][1] = P[1];

	/* Set process and measurement noise */
	ekf->Q[0] = Q[0];
	ekf->Q[1] = Q[1];

	ekf->R[0] = R[0];
	ekf->R[1] = R[1];
	ekf->R[2] = R[2];

}

void EKF_Predict(EKF *ekf, float p_rps, float q_rps, float r_rps, float sampleTime_s) {

	/* Pre-compute trigonometric quantities */
	float sp = sinf(ekf->phi_r);
	float cp = cosf(ekf->phi_r);
	float tt = tanf(ekf->theta_r);

	/* Compute state transition function dx/dt = f(x,u) */
	float dphidt	= p_rps + tt * (q_rps * sp + r_rps * cp);
	float dthetadt	= 				q_rps * cp - r_rps * sp;

	/* Update state estimates (x(n+1) = x(n) + T * dx/dt) */
	ekf->phi_r 		+= sampleTime_s * dphidt;
	ekf->theta_r	+= sampleTime_s * dthetadt;

	/* Re-compute trigonometric quantities */
	sp 			= sinf(ekf->phi_r);
	cp 			= cosf(ekf->phi_r);
	tt 			= tanf(ekf->theta_r);
	float ctInv = 1.0f / cosf(ekf->theta_r);

	/* Compute Jacobian of state transition function A(x,u) = df(x,u)/dx */
	float A[2][2] =
	{ 	{ tt * (q_rps * cp - r_rps * sp), (q_rps * sp + r_rps * cp) * ctInv * ctInv },
		{ -(q_rps * sp + r_rps * cp), 	  0.0f } };

	/* Update state covariance matrix P(n+1) = P(n) + T * (A * P(n) + P(n) * A' + Q) (note that A[1][1] = 0!) */
	float Pnew[2][2] =
	{	{A[0][0] * ekf->P[0][0] + A[0][1] * ekf->P[1][0] + ekf->P[0][0] * A[0][0] + ekf->P[0][1] * A[1][0] + ekf->Q[0], A[0][0] * ekf->P[0][1] + A[0][1] * ekf->P[1][1] + ekf->P[0][0] * A[0][1]},
		{A[1][0] * ekf->P[0][0] + 						   ekf->P[1][0] * A[0][0] + ekf->P[1][1] * A[1][0], 			A[1][0] * ekf->P[0][1] + 						  ekf->P[1][0] * A[0][1] + ekf->Q[1]} };

	ekf->P[0][0] += sampleTime_s * Pnew[0][0]; ekf->P[0][1] += sampleTime_s * Pnew[0][1];
	ekf->P[1][0] += sampleTime_s * Pnew[1][0]; ekf->P[1][1] += sampleTime_s * Pnew[1][1];

}

void EKF_Update(EKF *ekf, float ax_mps2, float ay_mps2, float az_mps2) {

	/* Normalise accelerometer readings */
	float accNormFactor = 1.0f / sqrtf(ax_mps2 * ax_mps2 + ay_mps2 * ay_mps2 + az_mps2 * az_mps2);

	float ax_norm 		= ax_mps2 * accNormFactor;
	float ay_norm 		= ay_mps2 * accNormFactor;
	float az_norm 		= az_mps2 * accNormFactor;

	/* Compute Jacobian of output function C(x,u) = dh(x,u)/dx */
	float sp = sinf(ekf->phi_r);
	float cp = cosf(ekf->phi_r);
	float st = sinf(ekf->theta_r);
	float ct = cosf(ekf->theta_r);


	float C[3][2] = {	{0.0f, 		ct},
						{-cp * ct, 	sp * st},
						{sp * ct, 	cp * st} };

	/* Compute Kalman gain K = P * C' * (R + C * P * C ')^-1 in steps (note that C[0][0] = 0!) */

	/* P * C'*/
	float PCt[2][3] = { {ekf->P[0][1] * C[0][1], ekf->P[0][0] * C[1][0] + ekf->P[0][1] * C[1][1], ekf->P[0][0] * C[2][0] + ekf->P[0][1] * C[2][1]},
						{ekf->P[1][1] * C[0][1], ekf->P[1][0] * C[1][0] + ekf->P[1][1] * C[1][1], ekf->P[1][0] * C[2][0] + ekf->P[1][1] * C[2][1]} };

	/* R + C * P * C' */
	float RCPCt[3][3] = { 	{C[0][1] * PCt[1][0] + ekf->R[0],			C[0][1] * PCt[1][1], 									C[0][1] * PCt[1][2]},
							{C[1][0] * PCt[0][0] + C[1][1] * PCt[1][0],	C[1][0] * PCt[0][1] + C[1][1] * PCt[1][1] + ekf->R[1],	C[1][0] * PCt[0][2] + C[1][1] * PCt[1][2]},
							{C[2][0] * PCt[0][0] + C[2][1] * PCt[1][0],	C[2][0] * PCt[0][1] + C[2][1] * PCt[1][1],				C[2][0] * PCt[0][2] + C[2][1] * PCt[1][2] + ekf->R[2]} };

	/* inv(R + C * P * C') */
	float detMatInv = 1.0f / (RCPCt[0][0] * (RCPCt[2][2] * RCPCt[1][1] - RCPCt[2][1] * RCPCt[1][2])
					- RCPCt[1][0] * (RCPCt[2][2] * RCPCt[0][1] - RCPCt[2][1] * RCPCt[0][2])
					+ RCPCt[2][0] * (RCPCt[1][2] * RCPCt[0][1] - RCPCt[1][1] * RCPCt[0][2]));

	float matInv[3][3] = { 	{	RCPCt[2][2] * RCPCt[1][1] - RCPCt[2][1] * RCPCt[1][2], -(	RCPCt[2][2] * RCPCt[0][1] - RCPCt[2][1] * RCPCt[0][2]), 	RCPCt[1][2] * RCPCt[0][1] - RCPCt[1][1] * RCPCt[0][2] },
							{-(	RCPCt[2][2] * RCPCt[1][0] - RCPCt[2][0] * RCPCt[1][2]), 	RCPCt[2][2] * RCPCt[0][0] - RCPCt[2][0] * RCPCt[0][2], -(	RCPCt[1][2] * RCPCt[0][0] - RCPCt[1][0] * RCPCt[0][2]) },
							{	RCPCt[2][1] * RCPCt[1][0] - RCPCt[2][0] * RCPCt[1][1], -(	RCPCt[2][1] * RCPCt[0][0] - RCPCt[2][0] * RCPCt[0][1]), 	RCPCt[1][1] * RCPCt[0][0] - RCPCt[1][0] * RCPCt[0][1]} };

	for (unsigned int i = 0; i < 3; i++) {

		for (unsigned int j = 0; j < 3; j++) {

			matInv[i][j] *= detMatInv;

		}

	}

	/* C' * inv(R + C * P * C') */
	float CtmatInv[2][3] = { 	{						  C[1][0] * matInv[1][0] + C[2][0] * matInv[2][0], 							C[1][0] * matInv[1][1] + C[2][0] * matInv[2][1], 						  C[1][0] * matInv[1][2] + C[2][0] * matInv[2][2]},
								{C[0][1] * matInv[0][0] + C[1][1] * matInv[1][0] + C[2][1] * matInv[2][0], C[0][1] * matInv[0][1] + C[1][1] * matInv[1][1] + C[2][1] * matInv[2][1], C[0][1] * matInv[0][2] + C[1][1] * matInv[1][2] + C[2][1] * matInv[2][2]} };

	/* K = P * C' * inv(R + C * P * C') */
	float K[2][3] = { 	{ekf->P[0][0] * CtmatInv[0][0] + ekf->P[0][1] * CtmatInv[1][0], ekf->P[0][0] * CtmatInv[0][1] + ekf->P[0][1] * CtmatInv[1][1], ekf->P[0][0] * CtmatInv[0][2] + ekf->P[0][1] * CtmatInv[1][2]},
						{ekf->P[1][0] * CtmatInv[0][0] + ekf->P[1][1] * CtmatInv[1][0], ekf->P[1][0] * CtmatInv[0][1] + ekf->P[1][1] * CtmatInv[1][1], ekf->P[1][0] * CtmatInv[0][2] + ekf->P[1][1] * CtmatInv[1][2]} };

	/* Update state covariance matrix P(n+1) = (I - K * C) * P(n) */
	float IminKC[2][2] = { 	{1.0f - (K[0][1] * C[1][0] + K[1][0] * C[2][0]), -(K[0][1] * C[1][1] + K[1][0] * C[2][1])},
							{-(K[1][1] * C[1][0] + K[1][2] * C[2][0]), 1.0f - (K[1][1] * C[1][1] + K[1][2] * C[2][1])} };

	float Pnew[2][2] = { 	{IminKC[0][0] * ekf->P[0][0] + IminKC[0][1] * ekf->P[1][0], IminKC[0][0] * ekf->P[0][1] + IminKC[0][1] * ekf->P[1][1]},
							{IminKC[1][0] * ekf->P[0][0] + IminKC[1][1] * ekf->P[1][0], IminKC[1][0] * ekf->P[0][1] + IminKC[1][1] * ekf->P[1][1]} };

	ekf->P[0][0] = Pnew[0][0]; ekf->P[0][1] = Pnew[0][1];
	ekf->P[1][0] = Pnew[1][0]; ekf->P[1][1] = Pnew[1][1];

	/* Compute output function h(x,u) */
	float h[3] = {	 sinf(ekf->theta_r),
					-cosf(ekf->theta_r) * sinf(ekf->phi_r),
					-cosf(ekf->theta_r) * cosf(ekf->phi_r) };

	/* Update state estimate x(n+1) = x(n) + K * (y - h) */
	ekf->phi_r 		= K[0][0] * (ax_norm - h[0]) + K[0][1] * (ay_norm - h[1]) + K[0][2] * (az_norm - h[2]);
	ekf->theta_r 	= K[1][0] * (ax_norm - h[0]) + K[1][1] * (ay_norm - h[1]) + K[1][2] * (az_norm - h[2]);

}
