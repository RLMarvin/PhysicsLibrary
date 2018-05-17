#ifndef VECTOR_HPP
#define VECTOR_HPP

#include <math.h>

struct Rotator;
struct Vector2;
struct Vector3;

#define CONST_UnrRotToRad                                        0.00009587379924285
#define CONST_RadToUnrRot                                        10430.3783504704527

struct Vector2
{
public:
	float X;
	float Y;

public:
	Vector2()
	{
		Vector2(0.0f, 0.0f);
	}

	Vector2(float xy)
	{
		Vector2(xy, xy);
	}

	Vector2(float x, float y
	)
	{
		X = x;
		Y = y;
	}

	float Length() const
	{
		return sqrt(X*X + Y*Y);
	}

	float Size() const
	{
		return this->Length();
	}

	Vector2& Normalize();
	float AngleTo(const Vector2& OtherVector) const;

	float operator * (const Vector2& DotFactor) const;
	Vector2 operator * (float factor) const;
	Vector2 Vector2::operator / (float divisor) const;
	Vector2 Vector2::operator + (const Vector2& Summand) const;
	Vector2 Vector2::operator - (const Vector2& Subtrahend) const;
	Vector2& Vector2::operator *= (float factor);
	Vector2& Vector2::operator /= (float divisor);
	Vector2& Vector2::operator += (const Vector2& Summand);
	Vector2& Vector2::operator -= (const Vector2& Subtrahend);
	bool Vector2::operator == (const Vector2& OtherVector) const;
	bool Vector2::operator != (const Vector2& OtherVector) const;

public:
	static Vector2 Normalize(const Vector2& TheVector);
	static float AngleBetween(Vector2& FirstVector, Vector2& SecondVector);
};

struct Vector3
{
public:
	float X;
	float Y;
	float Z;

public:
	Vector3()
	{
		Vector3(0.0f, 0.0f, 0.0f);
	}

	Vector3(float xyz)
	{
		Vector3(xyz, xyz, xyz);
	}

	Vector3(float x, float y, float z)
	{
		X = x;
		Y = y;
		Z = z;
	}

	float Length() const
	{
		return sqrt(X * X + Y * Y + Z * Z);
	}

	float Size() const
	{
		return Length();
	}

	Vector3& Normalize();

	float AngleTo(const Vector3& OtherVector) const;
	float operator * (const Vector3& DotFactor) const;

	Vector3 operator * (float factor) const;
	Vector3 operator / (float divisor) const;
	Vector3 operator + (const Vector3& Summand) const;
	Vector3 operator - (const Vector3& Subtrahend) const;

	Vector3& operator *= (float factor);
	Vector3& operator /= (float factor);
	Vector3& operator += (const Vector3& Summand);
	Vector3& operator -= (const Vector3& Subtrahend);
	bool operator == (const Vector3& OtherVector) const;
	bool operator != (const Vector3& OtherVector) const;

public:
	static Vector3 Normalize(const Vector3& TheVector);
	static float AngleBetween(Vector3& FirstVector, Vector3& SecondVector);
	Rotator ToRotation() const;

#ifdef COMPONENT_USE_D3DXSUPPORT
	static Vector3 D3DXVECTOR4ToVector3(D3DXVECTOR4& Input);
	static Vector3 D3DXVECTOR3ToVector3(D3DXVECTOR3& Input);

	D3DXVECTOR3 ToD3DXVECTOR3() const;
	D3DXVECTOR4 ToD3DXVECTOR4() const;
#endif
};


struct Rotator
{
public:
	int Pitch;
	int Yaw;
	int Roll;

private:
	static int NormalizeAxis(int angle);

public:
	Rotator()
	{
		Rotator(0);
	}

	Rotator(int pyr)
	{
		Rotator(pyr, pyr, pyr);
	}

	Rotator(int pitch, int yaw, int roll)
	{
		Pitch = pitch;
		Yaw = yaw;
		Roll = roll;
	}

	Rotator operator + (const Rotator& Summand) const;
	Rotator operator - (const Rotator& Subtrahend) const;

	Rotator& Normalize();
	Vector3 ToVector3() const;

public:
	static Rotator Normalize(const Rotator& TheRotator);
};


#endif