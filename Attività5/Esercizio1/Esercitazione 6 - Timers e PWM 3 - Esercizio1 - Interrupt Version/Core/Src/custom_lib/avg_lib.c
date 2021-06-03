/* Includes */
#include "custom_lib/avg_lib.h"

// ------------------ MEDIA MOBILE ------------------------
float movingAvg(float *ptrArrNumbers, float *ptrSum, int pos, int len, int nextNum){
	*ptrSum = *ptrSum - ptrArrNumbers[pos] + nextNum;
	ptrArrNumbers[pos] = nextNum;
	return *ptrSum / len;
}

void updateAvg(int *pos, int len, int *i, int count){
	(*pos)++;	//*pos = *pos + 1;
	if (*pos >= len){
		*pos = 0;
	}
	(*i)++;	//*i = *i + 1;
	if (*i >= count){
		*i = 0;
	}
}
