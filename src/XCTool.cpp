//#include "stdafx.h"
#include "XCTool.h"
#include <iostream>
using namespace std;
string FindDFileByRGB(vector<XCAssociationKey>& aKeyVec, string rgb) {
	vector<XCAssociationKey>::iterator iter;
	for (iter = aKeyVec.begin(); iter != aKeyVec.end();iter++) {
		
		if (iter->rgb == rgb) {
			return iter->d;
		}
		
	}
	abort();
}
