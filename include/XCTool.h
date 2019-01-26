#pragma once
#include <string>
#include <vector>
using std::string;
using std::vector;
class XCTool
{
public:

};

class XCKey {
public:
	string frameID;
	double tx, ty, tz;
	double qx, qy, qz, qw;
};

class XCAssociationKey {
public:
	string rgb, full_rgb, d, full_d;
};

class XCKITTIKey {
public:
	double r00, r01, r02, r10, r11, r12, r20, r21, r22;
	double tx, ty, tz;
};

string FindDFileByRGB(vector<XCAssociationKey>& aKeyVec, string rgb);

