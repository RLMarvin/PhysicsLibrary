#include "stdafx.h"
#include <iostream>
#include <Eigen/Dense>


#define DLLEXPORT extern "C" __declspec(dllexport)

DLLEXPORT void connect()
{
	std::cout << "Connected to C++ extension...\n";
}


using namespace std;
using namespace Eigen;


const float pi = 3.14159265359f;

const float drag = 0.03f;							// air resistance
const Vector3f gravity = { 0, 0, -650 };
const Vector3f z3 = { 0, 0, 0 };

const float e2 = 0.6f, e1 = 0.714f, a = 0.4f;       // ball bounce constants
const float R = 93;									// ball radius

const float wx = 8200, wy = 10240, wz = 2050;		// field dimensions
const float gx = 1792, gz = 640;					// goal dimensions
const float cR = 520, cR2 = 260, cR3 = 190;			// ramp radii
const float dR = 8060;								// diamond Radius

													/* circle/ramp locations */
const float cx = wx / 2 - cR, cy = wy / 2 - cR, cz = wz - cR;
const float cx2 = wx / 2 - cR2, cz2 = cR2;
const float cy3 = wy / 2 - cR3, cz3 = cR3;


// Structs

struct Vector2
{
	float X;
	float Y;
};


struct Vector3
{
	float X;
	float Y;
	float Z;
};


struct BallState
{
	Vector3 Location;
	Vector3 Velocity;
	Vector3 AngularVelocity;
};


struct BallPath
{
	BallState ballstates[999];
	int numstates = 999;
};

// end Structs


// Util

int Sign(const float x)
{
	if (x > 0)
		return 1;
	else
		return -1;
}


float Range(const float v, const float r)
{
	if (abs(v) > r)
		return r * Sign(v);
	else
		return v;
};


float pos(float x)
{
	if (x < 0)
		return 0;
	else
		return x;
}


float d3(const Eigen::Vector3f A)
{
	return sqrt(A[0] * A[0] + A[1] * A[1] + A[2] * A[2]);
}


Vector2 rotate2D(const float x, const float y, const float ang)
{
	Vector2 V;
	V.X = x * cos(ang) - y * sin(ang);
	V.Y = y * cos(ang) + x * sin(ang);
	return V;
}


float quadratic_pos(float a, float b, float c)
{
	float s = -1, s1 = -1, s2 = -1;
	if (a != 0 && b * b - 4 * a * c >= 0)
	{
		s1 = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
		s2 = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
		if (s1 < 0 && s2 < 0 || s1 * s2 < 0)
		{
			s = max(s1, s2);
		}
		else if (s1 > 0 && s2 > 0)
		{
			s = min(s1, s2);
		}

	}
	return pos(s);
}

// end Util


// Physics

Vector3f local_space(const Vector3f tL, const Vector3f oL, const Vector2f oR)
{
	Vector3f L = tL - oL;
	Vector2 tmp = rotate2D(L[1], L[2], -oR[0] * pi / 180);
	L[1] = tmp.X; L[2] = tmp.Y;
	tmp = rotate2D(L[0], L[2], -oR[1] * pi / 180);
	L[0] = tmp.X; L[2] = tmp.Y;
	return L;
}


Vector3f global_space(const Vector3f L, const Vector3f oL, const Vector2f oR)
{
	Vector3f tL;
	Vector2 tmp = rotate2D(L[0], L[2], oR[1] * pi / 180);
	tL[0] = tmp.X; tL[2] = tmp.Y;
	tmp = rotate2D(L[1], L[2], oR[0] * pi / 180);
	tL[1] = tmp.X; tL[2] = tmp.Y;
	return tL + oL;
}


bool CollisionFree(Vector3f L)
{
	if (242 < L[2] && L[2] < 1833)
	{
		if (abs(L[0]) < 3278)
		{
			if (abs(L[1]) < 4722)
			{
				if ((abs(L[0]) + abs(L[1])) / 7424 <= 1)
					return 1;
			}
		}
	}
	return 0;
}


struct ColState
{
	bool isCollided;
	Vector3f Location;	// Surface location
	Vector2f Rotation;	// Surface rotation, pitch, roll angles in degrees
};



ColState Collision_Model(Vector3f L)
{
	float x = L[0], y = L[1], z = L[2];
	ColState Cl;

	// Top Ramp X-axis
	if (abs(x) > wx / 2 - cR && z > cz && pow(abs(x) - cx, 2) + pow(z - cz, 2) > pow(cR - R, 2))
	{
		float a = atan2(z - cz, abs(x) - cx) / pi * 180;
		Cl.isCollided = true;
		Cl.Location = Vector3f{ cx * Sign(x), y, cz };
		Cl.Rotation = Vector2f{ 0, (90 + a) * Sign(x) };
		return Cl;
	}

	// Top Ramp Y-axis
	else if (abs(y) > cy && z > cz && pow(abs(y) - cy, 2) + pow(z - cz, 2) > pow(cR - R, 2))
	{
		float a = atan2(z - cz, abs(y) - cy) / pi * 180;
		Cl.isCollided = true;
		Cl.Location = Vector3f{ x, cy * Sign(y), cz };
		Cl.Rotation = Vector2f{ (90 + a) * Sign(y), 0 };
		return Cl;
	}

	// Bottom Ramp X-axis
	else if (abs(x) > cx2 && z < cz2 && pow(abs(x) - cx2, 2) + pow(z - cz2, 2) > pow(cR2 - R, 2))
	{
		float a = atan2(z - cz2, abs(x) - cx2) / pi * 180;
		Cl.isCollided = true;
		Cl.Location = Vector3f{ cx2 * Sign(x), y, cz2 };
		Cl.Rotation = Vector2f{ 0, (90 + a) * Sign(x) };
		return Cl;
	}

	// Bottom Ramp Y-axis
	else if (abs(y) > cy3 && z < cz3 && abs(x) > gx / 2 - R / 2 && pow(abs(y) - cy3, 2) + pow(z - cz2, 2) > pow(cR3 - R, 2))
	{
		float a = atan2(z - cz2, abs(y) - cy3) / pi * 180;
		Cl.isCollided = true;
		Cl.Location = Vector3f{ x, cy3 * Sign(y), cz3 };
		Cl.Rotation = Vector2f{ (90 + a) * Sign(y), 0 };
		return Cl;
	}

	// Flat 45° Corner
	else if ((abs(x) + abs(y) + R) / dR >= 1)
	{
		Vector3f dL{ dR * Sign(x), 0, 0 }; // a point in the diamond
		Cl.isCollided = true;
		Cl.Rotation = Vector2f{ 90 * Sign(y), 45 * Sign(x) };
		Vector3f sL = local_space(L, dL, Cl.Rotation); // Location in local space of the surface
		sL[2] = 0; // projection
		Cl.Location = global_space(sL, dL, Cl.Rotation);
		return Cl;
	}
	// Floor
	else if (z < R)
	{
		Cl.isCollided = true;
		Cl.Location = Vector3f{ x, y, 0 };
		Cl.Rotation = Vector2f{ 0, 0 };
		return Cl;
	}

	// Flat Wall X-axis
	else if (abs(x) > wx / 2 - R)
	{
		Cl.isCollided = true;
		Cl.Location = Vector3f{ wx * Sign(x), y, z };
		Cl.Rotation = Vector2f{ 0, 90 * Sign(x) };
		return Cl;
	}

	// Flat Wall Y-axis
	else if (abs(y) > wy / 2 - R && (abs(x) > gx / 2 - R / 2 || z > gz - R / 2))
	{
		Cl.isCollided = true;
		Cl.Location = Vector3f{ x, wy * Sign(y), z };
		Cl.Rotation = Vector2f{ 90 * Sign(y), 0 };
		return Cl;
	}

	// Ceiling
	else if (z > wz - R)
	{
		Cl.isCollided = true;
		Cl.Location = Vector3f{ x, y, wz };
		Cl.Rotation = Vector2f{ 0, 180 };
		return Cl;
	}

	// no collision
	else {
		Cl.isCollided = false;
		Cl.Location = Vector3f{ x, y, z };
		Cl.Rotation = Vector2f{ 0, 0 };
		return Cl;
	}
}


Vector3f simple_step(Vector3f L0, Vector3f V0, float dt)
{
	Vector3f Acc, nL, nV;

	Acc = gravity - 0.35 * drag * V0;
	nV = V0 + Acc * dt;
	nL = L0 + V0 * dt + 0.5 * Acc * dt * dt;
	return nL;
}


float time_solve_z(float z, float zv, float terminal_z, float g = gravity[2])
{
	float a = 0.35f * z * drag - 0.5f * g;
	float b = -zv;
	float c = -z + terminal_z;

	return quadratic_pos(a, b, c);
}


DLLEXPORT BallState step(BallState Ball, float dt, bool colCheck = true)
{

	Vector3f L0 = { Ball.Location.X, Ball.Location.Y, Ball.Location.Z };
	Vector3f V0 = { Ball.Velocity.X, Ball.Velocity.Y, Ball.Velocity.Z };
	Vector3f AV0 = { Ball.AngularVelocity.X, Ball.AngularVelocity.Y, Ball.AngularVelocity.Z };

	Vector3f Acc;
	Vector3f nL, nV, nAV;

	// simple step, no collision
	Acc = gravity - drag * V0;
	nV = V0 + Acc * dt;
	nL = L0 + V0 * dt + 0.5 * Acc * dt * dt;
	nAV = AV0;

	if (colCheck && !CollisionFree(nL))
	{

		ColState Cl = Collision_Model(nL);
		if (Cl.isCollided)
		{
			Vector3f lV, lAV;
			// transorforming velocities to local space
			lV = local_space(V0, z3, Cl.Rotation);
			lAV = local_space(AV0, z3, Cl.Rotation);
			float zv0 = lV[2];

			// bounce angle
			float ang = abs(atan2(lV[2], sqrt(pow(lV[0], 2) + pow(lV[1], 2)))) / pi * 180;

			// some more magic numbers
			float e = (e1 - .9915f) / (29) * ang + .9915f;

			// limiting e to range[e1, 1]
			if (e < e1)
				e = e1;

			bool rolling = false;
			if (abs(lV[2]) < 199)
				rolling = true;

			if (!rolling)
			{
				// bounce calculations
				lV = Vector3f{ (lV[0] + lAV[1] * R * a) * e, (lV[1] - lAV[0] * R * a) * e, abs(lV[2]) * e2 };
				lAV[0] = -lV[1] / R, lAV[1] = lV[0] / R;

				// limiting ball spin
				float total_av = d3(lAV);
				if (total_av > 6)
					lAV *= 6 / total_av;
			}
			else
				lV[2] = abs(lV[2])*e2;

			// transorforming velocities back to global/world space
			nV = global_space(lV, z3, Cl.Rotation);
			nAV = global_space(lAV, z3, Cl.Rotation);

			if (rolling)
				nV = nV * (1 - .6 * dt);

			// redoing the step with the new velocity
			Vector3f lL0 = local_space(L0, Cl.Location, Cl.Rotation);
			Vector3f lG = local_space(gravity, z3, Cl.Rotation);

			// small step towards contact point
			float cTime = Range(time_solve_z(lL0[2], zv0, R, lG[2]), dt);
			nL = simple_step(L0, V0, cTime);
			dt -= cTime;

			// continue step for what's left
			Acc = gravity - drag * nV;
			nV = nV + Acc * dt;
			nL = nL + nV * dt + 0.5 * Acc * dt * dt;
		}
	}

	// limiting ball speed
	float total_v = d3(nV);
	if (total_v > 6000)
	{
		nV *= 6000 / total_v;
	}

	Ball.Location = { nL[0], nL[1], nL[2] };
	Ball.Velocity = { nV[0], nV[1], nV[2] };
	Ball.AngularVelocity = { nAV[0], nAV[1], nAV[2] };

	return Ball;

}


DLLEXPORT BallPath predictPath(BallState Ball, float dt, float tps = 120)
{
	BallPath Path = {};
	Path.numstates = int(Range(abs(dt*tps), 999.0f));

	Path.ballstates[0] = Ball;

	// step-by-step simulation
	for (int i = 1; i < Path.numstates; i++)
	{
		Path.ballstates[i] = step(Path.ballstates[i - 1], 1 / tps);
	}

	return Path;
}

// end Physics




/*
// default and parameterized constructor
Vector3(float X = 0, float Y = 0, float Z = 0)
: X(X), Y(Y), Z(Z)
{}

// assignment operator
Vector3 & operator = (const Vector3 & A)
{
X = A.X;
Y = A.Y;
Z = A.Z;
return *this;
}

// add operator
Vector3 & operator + (const Vector3 & A) const
{
return Vector3(A.X + X, A.Y + Y, A.Z + Z);
}

// sub operator
Vector3 & operator - (const Vector3 & A) const
{
return Vector3(A.X - X, A.Y - Y, A.Z - Z);
}

// equality comparison
bool operator == (const Vector3 & A) const
{
return (X == A.X && Y == A.X && Z == A.Z);
}
*/

