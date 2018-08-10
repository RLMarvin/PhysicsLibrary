#include "stdafx.h"
#include <iostream>
#include "Vector.hpp"


#define DLLEXPORT extern "C" __declspec(dllexport)

DLLEXPORT void connect()
{
	std::cout << "Connected to C++ extension..." << std::endl;
}

const float PI = 3.141592653589793;
const float SQ2 = sqrt(2);

const float drag = 0.0306f;							// air resistance
const Vector3 z3 { 0, 0, 0 };
const float default_gc = -650;
const float default_cf = 300;

const float e2 = 0.6f;								// bounce factor
const float bR = 93;								// ball radius

const float wx = 4100, wy = 5120, wz = 2052;		// wall locations
const float gx = 1792, gz = 640;					// goal dimensions
const float cR = 520, cR2 = 260, cR3 = 190;			// ramp radii
const float dR = 8060;								// diamond Radius

// circle/ramp locations
const float cx = wx - cR, cy = wy - cR, cz = wz - cR;
const float cx2 = wx - cR2, cz2 = cR2;
const float cy3 = wy - cR3, cz3 = cR3;


// Structs


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
	Vector3 Location{0, 0, 0};	// Surface location
	Vector3 Rotation{0, 0, 0};	// Surface rotation, yaw, pitch, roll angles in degrees
};


struct InterceptState
{
	BallState Ball;
	CarState Car;
	float dt;
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


float distV3(Vector3 A, Vector3 B)
{
	return sqrt(pow(A.X - B.X, 2) + pow(A.Y - B.Y, 2) + pow(A.Z - B.Z, 2));
}


float dist3d(const Vector3 A)
{
	return sqrt(A.X * A.X + A.Y * A.Y + A.Z * A.Z);
}


float dist2d(const Vector2 A)
{
	return sqrt(A.X * A.X + A.Y * A.Y);
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

Vector3 local_space(const Vector3 tL, const Vector3 oL, const Vector3 oR)
{
	Vector3 L = tL - oL;
	Vector2 tmp = rotate2D(L.X, L.Y, -oR.X * PI / 180);
	L.X = tmp.X; L.Y = tmp.Y;
	tmp = rotate2D(L.Y, L.Z, -oR.Y * PI / 180);
	L.Y = tmp.X; L.Z = tmp.Y;
	tmp = rotate2D(L.X, L.Z, -oR.Z * PI / 180);
	L.X = tmp.X; L.Z = tmp.Y;
	return L;
}


Vector3 global_space(const Vector3 L, const Vector3 oL, const Vector3 oR)
{
	Vector3 tL{ 0, 0, 0 };
	Vector2 tmp = rotate2D(L.X, L.Z, oR.Z * PI / 180);
	tL.X = tmp.X; tL.Z = tmp.Y;
	tmp = rotate2D(L.Y, tL.Z, oR.Y * PI / 180);
	tL.Y = tmp.X; tL.Z = tmp.Y;
	tmp = rotate2D(tL.X, tL.Y, oR.X * PI / 180);
	tL.X = tmp.X; tL.Y = tmp.Y;
	return tL + oL;
}


bool CollisionFree(Vector3 L)
{
	if (242 < L.Z && L.Z < 1833)
	{
		if (abs(L.X) < 3278)
		{
			if (abs(L.Y) < 4722)
			{
				if ((abs(L.X) + abs(L.Y)) / 7424 <= 1)
					return 1;
			}
		}
	}
	return 0;
}



ColState Collision_Model(Vector3 L, float R = bR)
{
	float x = L.X, y = L.Y, z = L.Z;
	ColState Cl;

	// Top Ramp X-axis
	if (abs(x) > wx - cR && z > cz && pow(abs(x) - cx, 2) + pow(z - cz, 2) > pow(cR - R, 2))
	{
		float a = atan2(z - cz, abs(x) - cx);
		Cl.hasCollided = true;
		Cl.Location.X = (cR * cos(a) + cx) * Sign(x);
		Cl.Location.Y = y;
		Cl.Location.Z = cR * sin(a) + cz;
		Cl.Rotation.Z = (90.0f + a / PI * 180) * Sign(x);
		return Cl;
	}

	// Top Ramp Y-axis
	else if (abs(y) > cy && z > cz && pow(abs(y) - cy, 2) + pow(z - cz, 2) > pow(cR - R, 2))
	{
		float a = atan2(z - cz, abs(y) - cy);
		Cl.hasCollided = true;
		Cl.Location.X = x;
		Cl.Location.Y = (cR * cos(a) + cy) * Sign(y);
		Cl.Location.Z = cR * sin(a) + cz;
		Cl.Rotation.Y = (90.0f + a / PI * 180) * Sign(y);
		return Cl;
	}

	// Bottom Ramp X-axis
	else if (abs(x) > cx2 && z < cz2 && pow(abs(x) - cx2, 2) + pow(z - cz2, 2) > pow(cR2 - R, 2))
	{
		float a = atan2(z - cz2, abs(x) - cx2);
		Cl.hasCollided = true;
		Cl.Location.X = (cR2 * cos(a) + cx2) * Sign(x);
		Cl.Location.Y = y;
		Cl.Location.Z = cR2 * sin(a) + cz2;
		Cl.Rotation.Z = (90.0f + a / PI * 180) * Sign(x);
		return Cl;
	}

	// Bottom Ramp Y-axis
	else if (abs(y) > cy3 && z < cz3 && abs(x) > gx / 2 - R / 2 && pow(abs(y) - cy3, 2) + pow(z - cz3, 2) > pow(cR3 - R, 2))
	{
		float a = atan2(z - cz3, abs(y) - cy3);
		Cl.hasCollided = true;
		Cl.Location.X = x;
		Cl.Location.Y = (cR3 * cos(a) + cy3) * Sign(y);
		Cl.Location.Z = cR3 * sin(a) + cz3;
		Cl.Rotation.Y = (90.0f + a / PI * 180) * Sign(y);
		return Cl;
	}

	// 45° Top Ramp
	else if (abs(x) + abs(y) + R >= dR - cR && z > cz && pow(abs(x) + abs(y) - (dR - cR * SQ2), 2) + pow(z - cz2, 2) > pow(cR - R, 2))
	{
		float a = atan2(z - cz, abs(abs(x) + abs(y) - (dR - cR * SQ2)));
		Cl.hasCollided = true;
		Cl.Rotation.X = -45.0f * Sign(x) * Sign(y);
		Cl.Rotation.Y = (90.0f + a / PI * 180) * Sign(y);
		Vector3 oL { (dR - cR * SQ2) * Sign(x), 0, cz };  // circle origin
		Vector3 sL = local_space(L, oL, Cl.Rotation);
		sL.Z = -cR;
		Cl.Location = global_space(sL, oL, Cl.Rotation);
		return Cl;
	}
	
	// 45° Bottom Ramp
	else if (abs(x) + abs(y) + R >= dR - cR2 && z < cz2 && pow(abs(x) + abs(y) - (dR - cR2 * SQ2), 2) + pow(z - cz2, 2) > pow(cR2 - R, 2))
	{
		float a = atan2(z - cz2, abs(abs(x) + abs(y) - (dR - cR2 * SQ2)));
		Cl.hasCollided = true;
		Cl.Rotation.X = -45.0f * Sign(x) * Sign(y);
		Cl.Rotation.Y = (90.0f + a / PI * 180) * Sign(y);
		Vector3 oL { (dR - cR2 * SQ2) * Sign(x), 0, cR2 };  // circle origin
		Vector3 sL = local_space(L, oL, Cl.Rotation);
		sL.Z = -cR2;
		Cl.Location = global_space(sL, oL, Cl.Rotation);
		return Cl;
	}

	// Flat 45° Corner
	else if ((abs(x) + abs(y) + R) >= dR)
	{
		Cl.hasCollided = true;
		Cl.Rotation.X = -45.0f * Sign(x) * Sign(y);
		Cl.Rotation.Y = 90.0f * Sign(y);
		Vector3 dL { dR * Sign(x), 0, 0 }; // a point in the diamond
		Vector3 sL = local_space(L, dL, Cl.Rotation); // Location in local space of the surface
		sL.Z = 0; // projection
		Cl.Location = global_space(sL, dL, Cl.Rotation);
		return Cl;
	}
	// Floor
	else if (z < R)
	{
		Cl.hasCollided = true;
		Cl.Location.X = x;
		Cl.Location.Y = y;
		return Cl;
	}

	// Flat Wall X-axis
	else if (abs(x) > wx - R)
	{
		Cl.hasCollided = true;
		Cl.Location.X = wx * Sign(x);
		Cl.Location.Y = y;
		Cl.Location.Z = z;
		Cl.Rotation.Z = 90.0f * Sign(x);
		return Cl;
	}

	// Flat Wall Y-axis
	else if (abs(y) > wy - R && (abs(x) > gx / 2 - R / 2 || z > gz - R / 2))
	{
		Cl.hasCollided = true;
		Cl.Location.X = x;
		Cl.Location.Y = wy * Sign(y);
		Cl.Location.Z = z;
		Cl.Rotation.Y = 90.0f * Sign(x);
		return Cl;
	}

	// Ceiling
	else if (z > wz - R)
	{
		Cl.hasCollided = true;
		Cl.Location.X = x;
		Cl.Location.Y = y;
		Cl.Location.Z = wz;
		Cl.Rotation.Z = 180.0f;
		return Cl;
	}

	// no collision
	else {
		Cl.hasCollided = false;
		return Cl;
	}
}


Vector3 simple_step(Vector3 L0, Vector3 V0, float dt, Vector3 g)
{
	Vector3 Acc, nL, nV;

	Acc = g - V0 * 0.0202f;
	nV = V0 + Acc * dt;
	nL = L0 + V0 * dt + Acc * 0.5f * dt * dt;
	return nL;
}


float time_solve_z(float z, float zv, float terminal_z, float gc)
{
	float a = z * 0.0202 - 0.5f * gc;
	float b = -zv;
	float c = -z + terminal_z;

	return quadratic_pos(a, b, c);
}


DLLEXPORT BallState ballStep(BallState Ball, float dt, float gc = default_gc)
{	
	Vector3 Acc;
	Vector3 nL, nV, nAV = Ball.AngularVelocity;
	Vector3 gravity = { 0, 0, gc };

	// simple step, no collision
	Acc = gravity - Ball.Velocity * drag;
	nV = Ball.Velocity + Acc * dt;
	nL = Ball.Location + Ball.Velocity * dt +Acc * 0.5f * dt * dt;

	if (!CollisionFree(nL))
	{
		ColState Cl = Collision_Model(nL);
		if (Cl.hasCollided)
		{
			Vector3 lV, lAV, lL;

			// transorforming stuff to local space
			lV = local_space(Ball.Velocity, z3, Cl.Rotation);
			lAV = local_space(Ball.AngularVelocity, z3, Cl.Rotation);
			lL = local_space(Ball.Location, Cl.Location, Cl.Rotation);

			if (abs(lV.Z) > 1)
			{
				// small step towards contact point
				Vector3 lG = local_space(gravity, z3, Cl.Rotation);
				float cTime = Range(time_solve_z(lL.Z, lV.Z, bR, lG.Z), dt);
				lL = simple_step(lL, lV, cTime, lG);
				dt -= cTime;	
			}

			lL.Z = bR; // should be above surface

			Vector2 s = Vector2{lV.X, lV.Y} + Vector2{-lAV.Y * bR, lAV.X * bR};
			
			float p = min(2 * abs(lV.Z) / (dist2d(s) + 1e-9), 1) * 0.285;
			
			// applying bounce friction and spin
			lV.X -= s.X * p;
			lV.Y -= s.Y * p;

			// perpendicular bounce
			lV.Z = abs(lV.Z) * e2;

			// Angular velocity
			lAV.X = -lV.Y / bR;
			lAV.Y = lV.X / bR;

			// transorforming velocities back to global/world space
			nV = global_space(lV, z3, Cl.Rotation);
			nAV = global_space(lAV, z3, Cl.Rotation);
			nL = global_space(lL, Cl.Location, Cl.Rotation);

			// continue step for what's left
			Acc = gravity - nV * drag;
			nV = nV + Acc * dt;
			nL = nL + nV * dt + Acc * 0.5f * dt * dt;
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

	Ball.Location = nL;
	Ball.Velocity = nV;
	Ball.AngularVelocity = nAV;

	return Ball;

}


DLLEXPORT CarState carStep(CarState Car, float dt, float gc = default_gc, float cf = default_cf)
{
	Vector3 Acc;
	Vector3 nL, nV, nAV;
	Vector3 gravity = { 0, 0, gc };

	float t = max(dt, 0.0f);

	// simple step, no collision
	Acc = gravity - Car.Velocity * drag;
	nV = Car.Velocity + Acc * dt;
	nL = Car.Location + Car.Velocity * dt + Acc * 0.5f * dt * dt;

	if (!CollisionFree(nL))
	{
		ColState Cl = Collision_Model(nL, 50);
		if (Cl.hasCollided)
		{
			Vector3 lV, lAV, lL, lG;

			// transorforming stuff to local space
			lV = local_space(Car.Velocity, z3, Cl.Rotation);
			lL = local_space(Car.Location, Cl.Location, Cl.Rotation);
			lG = local_space(gravity, z3, Cl.Rotation);

			float total_v2d = sqrt(lV.X * lV.X + lV.Y * lV.Y);

			// surface friction
			if (total_v2d > 0.1f)
			{
				float f = max((total_v2d - cf * t) / total_v2d, 0.1f);
				lV.X *= f;
				lV.Y *= f;
			}

			// perpendicular bounce
			lV.Z = abs(lV.Z) * 0.05;

			lL.Z = 50; // above surface

			// transorforming velocities back to global/world space
			nV = global_space(lV, z3, Cl.Rotation);
			nAV = global_space(lAV, z3, Cl.Rotation);
			nL = global_space(lL, Cl.Location, Cl.Rotation);

			// continue step for after the bounce
			Acc = gravity - nV * drag;
			nV = nV + Acc * dt;
			nL = nL + nV * dt + Acc * 0.5f * dt * dt;
		}
	}

	// limiting Car speed
	float total_v = dist3d(nV);
	if (total_v > 2300)
	{
		nV *= 2300 / total_v;
	}

	Car.Location = nL;
	Car.Velocity = nV;

	return Car;

}


DLLEXPORT BallPath predictPath(BallState Ball, float dt, float tps = 120, float gc = default_gc)
{
	BallPath Path = {};
	Path.numstates = int(Range(abs(dt*tps), 999.0f));

	Path.ballstates[0] = Ball;

	// step-by-step simulation
	for (int i = 1; i < Path.numstates; i++)
	{
		Path.ballstates[i] = ballStep(Path.ballstates[i - 1], 1 / tps, gc);
	}

	return Path;
}


DLLEXPORT InterceptState intercept(BallState Ball, CarState Car, float maxdt, float tps = 120, float gc = default_gc, float cf = default_cf)
{
	BallState cBState = Ball, iBState = Ball;
	CarState cCState = Car, iCState = Car;

	float iTime;
	float sDist, cDist;

	// dt search
	int i = 0;
	
	while (i < min(maxdt * tps, 999) )
	{
		cBState = ballStep(cBState, 1 / tps, gc);
		cCState = carStep(cCState, 1 / tps, gc, cf);
		cDist = distV3(cBState.Location, cCState.Location);
		if (i == 0 || ((cDist < sDist)))
		{
			iBState = cBState;
			iCState = cCState;
			iTime = i / tps;
			sDist = cDist;
			if (sDist < bR)
				break;
		}
		i++;
	}

	
	InterceptState iState = { iBState, iCState, iTime };
	
	return iState;
}


// end Physics