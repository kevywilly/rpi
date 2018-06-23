/*
 * navigation.h
 *
 *  Created on: Jun 22, 2018
 *      Author: kevywilly
 */

#ifndef NAVIGATION_H_
#define NAVIGATION_H_

class Navigation {  

private:
	const static int threshold = 25;
	int minFL() {return FrontLeft < FrontLeft2 ? FrontLeft : FrontLeft2; }
	int minFR() {return FrontRight < FrontRight2 ? FrontRight : FrontRight2;}

public:

	int FrontLeft;
	int FrontRight;
	int FrontLeft2;
	int FrontRight2;
	int Left;
	int Right;
	int RearLeft;
	int RearRight;

	Navigation() {
		FrontLeft = 400;
		FrontRight = 400;
		FrontLeft2 = 30;
		FrontRight2 = 30;
		Left = 400;
		Right = 400;
		RearLeft = 30;
		RearRight = 30;
	}


	bool FrontIsClear() {
		return (FrontLeft >= 50 && FrontRight >= 50);
	}

	bool LeftIsClear() {
		return (Left >=15);
	}

	bool RightIsClear() {
		return (Right >= 15);
	}

	bool FrontRightIsClear() {
		return (FrontRight2 >= 15);
	}

	bool FrontLeftIsClear() {
		return (FrontLeft2 >= 15);
	}

	bool LeftIsGt() {
		return ((Left - 5) > Right);
	}

	bool RightIsGt() {
		return ((Right -5) > Left);
	}

	bool FrontLeftIsGt() {
		return((minFL() - 5) > minFR());
	}

	bool FrontRightIsGt() {
		return((minFR() - 5) > minFL());
	}

	bool RearRightIsClear() {
		return RearRight > 5;
	}

	bool RearLeftIsClear() {
		return RearLeft > 5;
	}
};



#endif /* NAVIGATION_H_ */