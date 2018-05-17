#include "stdafx.h"
#include "Vector.hpp"

Vector2& Vector2::Normalize()
{
	float size = this->Size();

	if (size == 0.0f)
	{
		this->X = 1.0f;
		this->Y = 1.0f;
	}
	else
		*this /= size;

	return *this;
}

float Vector2::AngleTo(const Vector2& OtherVector) const
{
	return acos((*this * OtherVector) / (this->Size() * OtherVector.Size()));
}

float Vector2::operator * (const Vector2& DotFactor) const
{
	return (this->X * DotFactor.X + this->Y * DotFactor.Y);
}

Vector2 Vector2::operator * (float factor) const
{
	Vector2 Result;
	Result.X = this->X * factor;
	Result.Y = this->Y * factor;

	return Result;
}

Vector2 Vector2::operator / (float divisor) const
{
	Vector2 Result;
	Result.X = this->X / divisor;
	Result.Y = this->Y / divisor;

	return Result;
}

Vector2 Vector2::operator + (const Vector2& Summand) const
{
	Vector2 Result;
	Result.X = this->X + Summand.X;
	Result.Y = this->Y + Summand.Y;

	return Result;
}

Vector2 Vector2::operator - (const Vector2& Subtrahend) const
{
	Vector2 Result;
	Result.X = this->X - Subtrahend.X;
	Result.Y = this->Y - Subtrahend.Y;

	return Result;
}

Vector2& Vector2::operator *= (float factor)
{
	*this = *this * factor;

	return *this;
}

Vector2& Vector2::operator /= (float divisor)
{
	*this = *this / divisor;

	return *this;
}

Vector2& Vector2::operator += (const Vector2& Summand)
{
	*this = *this + Summand;

	return *this;
}

Vector2& Vector2::operator -= (const Vector2& Subtrahend)
{
	*this = *this - Subtrahend;

	return *this;
}

bool Vector2::operator == (const Vector2& OtherVector) const
{
	return ((this->X == OtherVector.X) && (this->Y == OtherVector.Y));
}

bool Vector2::operator != (const Vector2& OtherVector) const
{
	return !(*this == OtherVector);
}

Vector2 Vector2::Normalize(const Vector2& TheVector)
{
	Vector2 Result = Vector2(TheVector.X, TheVector.Y);
	Result.Normalize();

	return Result;
}

float Vector2::AngleBetween(Vector2& FirstVector, Vector2& SecondVector)
{
	return acos((FirstVector * SecondVector) / (FirstVector.Size() * SecondVector.Size()));
}

Vector3& Vector3::Normalize()
{
	float size = this->Size();

	if (size == 0.0f)
	{
		this->X = 1.0f;
		this->Y = 1.0f;
		this->Z = 1.0f;
	}
	else
		*this /= size;

	return *this;
}

float Vector3::AngleTo(const Vector3& OtherVector) const
{
	return acos((*this * OtherVector) / (this->Size() * OtherVector.Size()));
}

float Vector3::operator * (const Vector3& DotFactor) const
{
	return (this->X * DotFactor.X + this->Y * DotFactor.Y + this->Z * DotFactor.Z);
}

Vector3 Vector3::operator * (float factor) const
{
	Vector3 Result;
	Result.X = this->X * factor;
	Result.Y = this->Y * factor;
	Result.Z = this->Z * factor;

	return Result;
}

Vector3 Vector3::operator / (float divisor) const
{
	Vector3 Result;
	Result.X = this->X / divisor;
	Result.Y = this->Y / divisor;
	Result.Z = this->Z / divisor;

	return Result;
}

Vector3 Vector3::operator + (const Vector3& Summand) const
{
	Vector3 Result;
	Result.X = this->X + Summand.X;
	Result.Y = this->Y + Summand.Y;
	Result.Z = this->Z + Summand.Z;

	return Result;
}

Vector3 Vector3::operator - (const Vector3& Subtrahend) const
{
	Vector3 Result;
	Result.X = this->X - Subtrahend.X;
	Result.Y = this->Y - Subtrahend.Y;
	Result.Z = this->Z - Subtrahend.Z;

	return Result;
}

Vector3& Vector3::operator *= (float factor)
{
	*this = *this * factor;

	return *this;
}

Vector3& Vector3::operator /= (float divisor)
{
	*this = *this / divisor;

	return *this;
}

Vector3& Vector3::operator += (const Vector3& Summand)
{
	*this = *this + Summand;

	return *this;
}

Vector3& Vector3::operator -= (const Vector3& Subtrahend)
{
	*this = *this - Subtrahend;

	return *this;
}

bool Vector3::operator == (const Vector3& OtherVector) const
{
	return ((this->X == OtherVector.X) && (this->Y == OtherVector.Y) && (this->Z == OtherVector.Z));
}

bool Vector3::operator != (const Vector3& OtherVector) const
{
	return !(*this == OtherVector);
}

Vector3 Vector3::Normalize(const Vector3& TheVector)
{
	Vector3 Result = Vector3(TheVector.X, TheVector.Y, TheVector.Z);
	Result.Normalize();

	return Result;
}

float Vector3::AngleBetween(Vector3& FirstVector, Vector3& SecondVector)
{
	return acos((FirstVector * SecondVector) / (FirstVector.Size() * SecondVector.Size()));
}

Rotator Vector3::ToRotation() const
{
	Rotator Rotation;
	Rotation.Yaw = (int)(atan2(this->Y, this->X) * CONST_RadToUnrRot);
	Rotation.Pitch = (int)(atan2(this->Z, sqrt((this->X * this->X) + (this->Y * this->Y))) * CONST_RadToUnrRot);
	Rotation.Roll = 0;

	return Rotation;
}


int Rotator::NormalizeAxis(int angle)
{
	angle &= 0xFFFF;

	if (angle > 32767)
		angle -= 0x10000;

	return angle;
}

Rotator Rotator::operator + (const Rotator& Summand) const
{
	Rotator Result;
	Result.Pitch = this->Pitch + Summand.Pitch;
	Result.Yaw = this->Yaw + Summand.Yaw;
	Result.Roll = this->Roll + Summand.Roll;

	return Result;
}

Rotator Rotator::operator - (const Rotator& Subtrahend) const
{
	Rotator Result;
	Result.Pitch = this->Pitch - Subtrahend.Pitch;
	Result.Yaw = this->Yaw - Subtrahend.Yaw;
	Result.Roll = this->Roll - Subtrahend.Roll;

	return Result;
}

Rotator& Rotator::Normalize()
{
	this->Pitch = NormalizeAxis(this->Pitch);
	this->Roll = NormalizeAxis(this->Roll);
	this->Yaw = NormalizeAxis(this->Yaw);

	return *this;
}

Vector3 Rotator::ToVector3() const
{
	Vector3 Result;
	float fYaw = (float)(this->Yaw * CONST_UnrRotToRad);
	float fPitch = (float)(this->Pitch * CONST_UnrRotToRad);
	float fCosPitch = cos(fPitch);
	Result.X = (float)cos(fYaw) * fCosPitch;
	Result.Y = (float)sin(fYaw) * fCosPitch;
	Result.Z = (float)sin(fPitch);

	return Result;
}

Rotator Rotator::Normalize(const Rotator& TheRotator)
{
	Rotator Result = Rotator(TheRotator.Pitch, TheRotator.Roll, TheRotator.Yaw);
	Result.Normalize();

	return Result;
}