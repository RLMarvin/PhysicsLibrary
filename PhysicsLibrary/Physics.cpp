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

const float drag = 0.0306f;							// air resistance
const Vector3f gravity = { 0, 0, -650 };
const Vector3f z3 = { 0, 0, 0 };

const float e2 = 0.6f, e1 = 0.714f, a = 0.4f;       // ball bounce constants
const float bR = 93;								// ball radius

const float wx = 4110, wy = 5120, wz = 2052;		// wall locations
const float gx = 1792, gz = 640;					// goal dimensions
const float cR = 520, cR2 = 260, cR3 = 240;			// ramp radii
const float dR = 8060;								// diamond Radius

// circle/ramp locations
const float cx = wx - cR, cy = wy - cR, cz = wz - cR;
const float cx2 = wx - cR2, cz2 = cR2;
const float cy3 = wy - cR3, cz3 = cR3;


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


struct CarState
{
	Vector3 Location;
	Vector3 Velocity;
};


struct ColState
{
	bool hasCollided;
	Vector3f Location;	// Surface location
	Vector2f Rotation;	// Surface rotation, pitch, roll angles in degrees
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


float dist3d(const Eigen::Vector3f A)
{
	return sqrt(A[0] * A[0] + A[1] * A[1] + A[2] * A[2]);
}


float dist2d(const Eigen::Vector3f A)
{
	return sqrt(A[0] * A[0] + A[1] * A[1]);
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
	return s;
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
	Vector3f tL{ 0, 0, 0 };
	Vector2 tmp = rotate2D(L[0], L[2], oR[1] * pi / 180);
	tL[0] = tmp.X; tL[2] = tmp.Y;
	tmp = rotate2D(L[1], tL[2], oR[0] * pi / 180);
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



ColState Collision_Model(Vector3f L, float R = bR)
{
	float x = L[0], y = L[1], z = L[2];
	ColState Cl;

	// Top Ramp X-axis
	if (abs(x) > wx - cR && z > cz && pow(abs(x) - cx, 2) + pow(z - cz, 2) > pow(cR - R, 2))
	{
		float a = atan2(z - cz, abs(x) - cx);
		Cl.hasCollided = true;
			Cl.Location[0] = (cR * cos(a) + cx) * Sign(x);
			Cl.Location[1] = y;
			Cl.Location[2] = cR * sin(a) + cz;
		Cl.Rotation = Vector2f{ 0, (90 + a / pi * 180) * Sign(x) };
		return Cl;
	}

	// Top Ramp Y-axis
	else if (abs(y) > cy && z > cz && pow(abs(y) - cy, 2) + pow(z - cz, 2) > pow(cR - R, 2))
	{
		float a = atan2(z - cz, abs(y) - cy);
		Cl.hasCollided = true;
		Cl.Location[0] = x;
		Cl.Location[1] = (cR * cos(a) + cy) * Sign(y);
		Cl.Location[2] = cR * sin(a) + cz;
		Cl.Rotation = Vector2f{ (90 + a / pi * 180) * Sign(y), 0 };
		return Cl;
	}

	// Bottom Ramp X-axis
	else if (abs(x) > cx2 && z < cz2 && pow(abs(x) - cx2, 2) + pow(z - cz2, 2) > pow(cR2 - R, 2))
	{
		float a = atan2(z - cz2, abs(x) - cx2);
		Cl.hasCollided = true;
		Cl.Location[0] = (cR2 * cos(a) + cx2) * Sign(x);
		Cl.Location[1] = y;
		Cl.Location[2] = cR2 * sin(a) + cz2;
		Cl.Rotation = Vector2f{ 0, (90 + a / pi * 180) * Sign(x) };
		return Cl;
	}

	// Bottom Ramp Y-axis
	else if (abs(y) > cy3 && z < cz3 && abs(x) > gx / 2 - R / 2 && pow(abs(y) - cy3, 2) + pow(z - cz2, 2) > pow(cR3 - R, 2))
	{
		float a = atan2(z - cz2, abs(y) - cy3);
		Cl.hasCollided = true;
		Cl.Location[0] = x;
		Cl.Location[1] = (cR3 * cos(a) + cy3) * Sign(y);
		Cl.Location[2] = cR3 * sin(a) + cz3;
		Cl.Rotation = Vector2f{ (90 + a / pi * 180) * Sign(y), 0 };
		return Cl;
	}

	// Flat 45° Corner
	else if ((abs(x) + abs(y) + R) >= dR)
	{
		Vector3f dL{ dR * Sign(x), 0, 0 }; // a point in the diamond
		Cl.hasCollided = true;
		Cl.Rotation = Vector2f{ 90 * Sign(y), 45 * Sign(x) };
		Vector3f sL = local_space(L, dL, Cl.Rotation); // Location in local space of the surface
		sL[2] = 0; // projection
		Cl.Location = global_space(sL, dL, Cl.Rotation);
		return Cl;
	}
	// Floor
	else if (z < R)
	{
		Cl.hasCollided = true;
		Cl.Location = Vector3f{ x, y, 0 };
		Cl.Rotation = Vector2f{ 0, 0 };
		return Cl;
	}

	// Flat Wall X-axis
	else if (abs(x) > wx - R)
	{
		Cl.hasCollided = true;
		Cl.Location = Vector3f{ wx * Sign(x), y, z };
		Cl.Rotation = Vector2f{ 0, 90 * Sign(x) };
		return Cl;
	}

	// Flat Wall Y-axis
	else if (abs(y) > wy - R && (abs(x) > gx / 2 - R / 2 || z > gz - R / 2))
	{
		Cl.hasCollided = true;
		Cl.Location = Vector3f{ x, wy* Sign(y), z };
		Cl.Rotation = Vector2f{ 90 * Sign(y), 0 };
		return Cl;
	}

	// Ceiling
	else if (z > wz - R)
	{
		Cl.hasCollided = true;
		Cl.Location = Vector3f{ x, y, wz };
		Cl.Rotation = Vector2f{ 0, 180 };
		return Cl;
	}

	// no collision
	else {
		Cl.hasCollided = false;
		Cl.Location = Vector3f{ x, y, z };
		Cl.Rotation = Vector2f{ 0, 0 };
		return Cl;
	}
}


Vector3f simple_step(Vector3f L0, Vector3f V0, float dt, Vector3f g=gravity)
{
	Vector3f Acc, nL, nV;

	Acc = g - 0.0202 * V0;
	nV = V0 + Acc * dt;
	nL = L0 + V0 * dt + 0.5 * Acc * dt * dt;
	return nL;
}


float time_solve_z(float z, float zv, float terminal_z, float g = gravity[2])
{
	float a = z * 0.0202 - 0.5f * g;
	float b = -zv;
	float c = -z + terminal_z;

	return quadratic_pos(a, b, c);
}


DLLEXPORT BallState ballStep(BallState Ball, float dt)
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

	if (!CollisionFree(nL))
	{
		ColState Cl = Collision_Model(nL);
		if (Cl.hasCollided)
		{
			Vector3f lV, lAV, lL;

			// transorforming stuff to local space
			lV = local_space(V0, z3, Cl.Rotation);
			lAV = local_space(AV0, z3, Cl.Rotation);
			lL = local_space(L0, Cl.Location, Cl.Rotation);

			float total_v = dist3d(lV);
			float total_v2d = dist2d(lV);

			if (abs(lV[2]) > 5) // if bouncing
			{
				float e = e1;

				if (total_v2d != 0)
				{
					e = (1 - abs(lV[2]) / total_v) * 0.6103 + 0.3962;
					// e = 1 - 0.5523438 * abs(lV[2]) / total_v2d;
					e = min(max(e, e1), .99);  // limiting e to range[e1, .99]
				}
				else
					e = e1;

				if (abs(lV[2]) < 210)
					lAV *= 0; // dont apply spin

				// applying bounce friction and spin
				lV[0] = (lV[0] + lAV[1] * bR * a) * e;
				lV[1] = (lV[1] - lAV[0] * bR * a) * e;

				// small step towards contact point
				Vector3f lG = local_space(gravity, z3, Cl.Rotation);
				float cTime = Range(time_solve_z(lL[2], lV[2], bR, lG[2]), dt);
				lL = simple_step(lL, lV, cTime, lG);
				dt -= cTime;
			}

			// perpendicular bounce
			lV[2] = abs(lV[2])*e2;

			// rolling calculations
			if (total_v2d > 565)
			{
				float f = (total_v2d - 230 * dt) / total_v2d;
				lV[0] *= f;
                lV[1] *= f;
			}	

			// Angular velocity calculations
			lAV[0] = -lV[1] / bR;
			lAV[1] = lV[0] / bR;

			lL[2] = bR; // above surface

			// transorforming velocities back to global/world space
			nV = global_space(lV, z3, Cl.Rotation);
			nAV = global_space(lAV, z3, Cl.Rotation);
			nL = global_space(lL, Cl.Location, Cl.Rotation);

			// continue step for what's left
			Acc = gravity - drag * nV;
			nV = nV + Acc * dt;
			nL = nL + nV * dt + 0.5 * Acc * dt * dt;
		}
	}

	// limiting ball spin
	float total_av = dist3d(nAV);
	if (total_av > 6)
		nAV *= 6 / total_av;

	// limiting ball speed
	float total_v = dist3d(nV);
	if (total_v > 6000)
		nV *= 6000 / total_v;

	Ball.Location = { nL[0], nL[1], nL[2] };
	Ball.Velocity = { nV[0], nV[1], nV[2] };
	Ball.AngularVelocity = { nAV[0], nAV[1], nAV[2] };

	return Ball;

}



DLLEXPORT CarState carStep(CarState Car, float dt)
{

	Vector3f L0 = { Car.Location.X, Car.Location.Y, Car.Location.Z };
	Vector3f V0 = { Car.Velocity.X, Car.Velocity.Y, Car.Velocity.Z };

	Vector3f Acc;
	Vector3f nL, nV, nAV;

	// simple step, no collision
	Acc = gravity - drag * V0;
	nV = V0 + Acc * dt;
	nL = L0 + V0 * dt + 0.5 * Acc * dt * dt;

	if (!CollisionFree(nL))
	{
		ColState Cl = Collision_Model(nL, 50);
		if (Cl.hasCollided)
		{
			Vector3f lV, lAV, lL, lG;

			// transorforming stuff to local space
			lV = local_space(V0, z3, Cl.Rotation);
			lL = local_space(L0, Cl.Location, Cl.Rotation);
			lG = local_space(gravity, z3, Cl.Rotation);

			lL[2] = 50;
			nL = global_space(lL, Cl.Location, Cl.Rotation);

			if (lV[2] > 199)
				lV *= 0.9;
			else
				lV = nV*(1 - 0.6 * dt);

			lV[2] = abs(lV[2]) * 0.1;

			// transorforming velocities back to global/world space
			nV = global_space(lV, z3, Cl.Rotation);
			nAV = global_space(lAV, z3, Cl.Rotation);

			// continue step for after the bounce
			Acc = gravity - drag * nV;
			nV = nV + Acc * dt;
			nL = nL + nV * dt + 0.5 * Acc * dt * dt;
		}
	}

	// limiting Car speed
	float total_v = dist3d(nV);
	if (total_v > 2300)
	{
		nV *= 2300 / total_v;
	}

	Car.Location = { nL[0], nL[1], nL[2] };
	Car.Velocity = { nV[0], nV[1], nV[2] };

	return Car;

}




DLLEXPORT BallPath predictPath(BallState Ball, float dt, float tps = 120)
{
	BallPath Path = {};
	Path.numstates = int(Range(abs(dt*tps), 999.0f));

	Path.ballstates[0] = Ball;

	// step-by-step simulation
	for (int i = 1; i < Path.numstates; i++)
	{
		Path.ballstates[i] = ballStep(Path.ballstates[i - 1], 1 / tps);
	}

	return Path;
}


DLLEXPORT BallState interceptState(BallState Ball, CarState Car, float maxdt, float tps = 120)
{
	int i = 0;
	BallState cBState = Ball, iState = Ball;
	CarState cCState = Car;

	while (i < Range(maxdt*tps, 999))
	{
		cBState = ballStep(cBState, 1 / tps);
		cCState = carStep(cCState, 1 / tps);
		i++;
	}

	return cBState;
}


DLLEXPORT float * return1(float * arr)
{
	for (int i = 0; i < 3; i++)
	{
		arr[i] = 3.14f;
		cout << arr[i] << endl;
	}
	
	return arr;
}

// end Physics
