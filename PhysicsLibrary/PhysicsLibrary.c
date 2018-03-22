#include <stdio.h>
#include <stdlib.h>

void main()
{
	return;
}

__declspec(dllexport) void connect()
{
	printf("Connected to C extension...\n");
}

//return random value in range of 0-50
__declspec(dllexport) int randNum()
{
	int nRand = rand() % 50;
	return nRand;
}

//add two number and return value
__declspec(dllexport) int addNum(int a, int b)
{
	int nAdd = a + b;
	return nAdd;
}