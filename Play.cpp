#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <iostream>
#include <windows.h>
#include <stdio.h>
#include <fstream>
#include<string>
#include <iomanip>

using namespace cv;
using namespace std;
int main() {
	
	//HANDLE hConsoleColor;
	
	int num_files = 200;
	int width = 28, height = 28;
	int i = 0;
	Mat dst1, dst3, dst4;
	Mat dst2;
	Mat edges, edges3, edges4;
	Mat edges2;
	Mat frame;
	Mat image[200];
	image[0] = imread("Tram1\\COBOT\\DATA_0.png", 0);
	image[1] = imread("Tram1\\COBOT\\DATA_1.png", 0);
	image[2] = imread("Tram1\\COBOT\\DATA_2.png", 0);
	image[3] = imread("Tram1\\COBOT\\DATA_3.png", 0);
	image[4] = imread("Tram1\\COBOT\\DATA_4.png", 0);
	image[5] = imread("Tram1\\COBOT\\DATA_5.png", 0);
	image[6] = imread("Tram1\\COBOT\\DATA_6.png", 0);
	image[7] = imread("Tram1\\COBOT\\DATA_7.png", 0);
	image[8] = imread("Tram1\\COBOT\\DATA_8.png", 0);
	image[9] = imread("Tram1\\COBOT\\DATA_9.png", 0);
	image[10] = imread("Tram1\\COBOT\\DATA_10.png", 0);
	image[11] = imread("Tram1\\COBOT\\DATA_11.png", 0);
	image[12] = imread("Tram1\\COBOT\\DATA_12.png", 0);
	image[13] = imread("Tram1\\COBOT\\DATA_13.png", 0);
	image[14] = imread("Tram1\\COBOT\\DATA_14.png", 0);
	image[15] = imread("Tram1\\COBOT\\DATA_15.png", 0);
	image[16] = imread("Tram1\\COBOT\\DATA_16.png", 0);
	image[17] = imread("Tram1\\COBOT\\DATA_17.png", 0);
	image[18] = imread("Tram1\\COBOT\\DATA_18.png", 0);
	image[19] = imread("Tram1\\COBOT\\DATA_19.png", 0);
	image[20] = imread("Tram1\\COBOT\\DATA_20.png", 0);
	image[21] = imread("Tram1\\COBOT\\DATA_21.png", 0);
	image[22] = imread("Tram1\\COBOT\\DATA_22.png", 0);
	image[23] = imread("Tram1\\COBOT\\DATA_23.png", 0);
	image[24] = imread("Tram1\\COBOT\\DATA_24.png", 0);
	image[25] = imread("Tram2\\COBOT\\DATA_0.png", 0);
	image[26] = imread("Tram2\\COBOT\\DATA_1.png", 0);
	image[27] = imread("Tram2\\COBOT\\DATA_2.png", 0);
	image[28] = imread("Tram2\\COBOT\\DATA_3.png", 0);
	image[29] = imread("Tram2\\COBOT\\DATA_4.png", 0);
	image[30] = imread("Tram2\\COBOT\\DATA_5.png", 0);
	image[31] = imread("Tram2\\COBOT\\DATA_6.png", 0);
	image[32] = imread("Tram2\\COBOT\\DATA_7.png", 0);
	image[33] = imread("Tram2\\COBOT\\DATA_8.png", 0);
	image[34] = imread("Tram2\\COBOT\\DATA_9.png", 0);
	image[35] = imread("Tram2\\COBOT\\DATA_10.png", 0);
	image[36] = imread("Tram2\\COBOT\\DATA_11.png", 0);
	image[37] = imread("Tram2\\COBOT\\DATA_12.png", 0);
	image[38] = imread("Tram2\\COBOT\\DATA_13.png", 0);
	image[39] = imread("Tram2\\COBOT\\DATA_14.png", 0);
	image[40] = imread("Tram2\\COBOT\\DATA_15.png", 0);
	image[41] = imread("Tram2\\COBOT\\DATA_16.png", 0);
	image[42] = imread("Tram2\\COBOT\\DATA_17.png", 0);
	image[43] = imread("Tram2\\COBOT\\DATA_18.png", 0);
	image[44] = imread("Tram2\\COBOT\\DATA_19.png", 0);
	image[45] = imread("Tram2\\COBOT\\DATA_20.png", 0);
	image[46] = imread("Tram2\\COBOT\\DATA_21.png", 0);
	image[47] = imread("Tram2\\COBOT\\DATA_22.png", 0);
	image[48] = imread("Tram2\\COBOT\\DATA_23.png", 0);
	image[49] = imread("Tram2\\COBOT\\DATA_24.png", 0);
	image[50] = imread("Tram3\\COBOT\\DATA_0.png", 0);
	image[51] = imread("Tram3\\COBOT\\DATA_1.png", 0);
	image[52] = imread("Tram3\\COBOT\\DATA_2.png", 0);
	image[53] = imread("Tram3\\COBOT\\DATA_3.png", 0);
	image[54] = imread("Tram3\\COBOT\\DATA_4.png", 0);
	image[55] = imread("Tram3\\COBOT\\DATA_5.png", 0);
	image[56] = imread("Tram3\\COBOT\\DATA_6.png", 0);
	image[57] = imread("Tram3\\COBOT\\DATA_7.png", 0);
	image[58] = imread("Tram3\\COBOT\\DATA_8.png", 0);
	image[59] = imread("Tram3\\COBOT\\DATA_9.png", 0);
	image[60] = imread("Tram3\\COBOT\\DATA_10.png", 0);
	image[61] = imread("Tram3\\COBOT\\DATA_11.png", 0);
	image[62] = imread("Tram3\\COBOT\\DATA_12.png", 0);
	image[63] = imread("Tram3\\COBOT\\DATA_13.png", 0);
	image[64] = imread("Tram3\\COBOT\\DATA_14.png", 0);
	image[65] = imread("Tram3\\COBOT\\DATA_15.png", 0);
	image[66] = imread("Tram3\\COBOT\\DATA_16.png", 0);
	image[67] = imread("Tram3\\COBOT\\DATA_17.png", 0);
	image[68] = imread("Tram3\\COBOT\\DATA_18.png", 0);
	image[69] = imread("Tram3\\COBOT\\DATA_19.png", 0);
	image[70] = imread("Tram3\\COBOT\\DATA_20.png", 0);
	image[71] = imread("Tram3\\COBOT\\DATA_21.png", 0);
	image[72] = imread("Tram3\\COBOT\\DATA_22.png", 0);
	image[73] = imread("Tram3\\COBOT\\DATA_23.png", 0);
	image[74] = imread("Tram3\\COBOT\\DATA_24.png", 0);
	image[75] = imread("Tram4\\COBOT\\DATA_0.png", 0);
	image[76] = imread("Tram4\\COBOT\\DATA_1.png", 0);
	image[77] = imread("Tram4\\COBOT\\DATA_2.png", 0);
	image[78] = imread("Tram4\\COBOT\\DATA_3.png", 0);
	image[79] = imread("Tram4\\COBOT\\DATA_4.png", 0);
	image[80] = imread("Tram4\\COBOT\\DATA_5.png", 0);
	image[81] = imread("Tram4\\COBOT\\DATA_6.png", 0);
	image[82] = imread("Tram4\\COBOT\\DATA_7.png", 0);
	image[83] = imread("Tram4\\COBOT\\DATA_8.png", 0);
	image[84] = imread("Tram4\\COBOT\\DATA_9.png", 0);
	image[85] = imread("Tram4\\COBOT\\DATA_10.png", 0);
	image[86] = imread("Tram4\\COBOT\\DATA_11.png", 0);
	image[87] = imread("Tram4\\COBOT\\DATA_12.png", 0);
	image[88] = imread("Tram4\\COBOT\\DATA_13.png", 0);
	image[89] = imread("Tram4\\COBOT\\DATA_14.png", 0);
	image[90] = imread("Tram4\\COBOT\\DATA_15.png", 0);
	image[91] = imread("Tram4\\COBOT\\DATA_16.png", 0);
	image[92] = imread("Tram4\\COBOT\\DATA_17.png", 0);
	image[93] = imread("Tram4\\COBOT\\DATA_18.png", 0);
	image[94] = imread("Tram4\\COBOT\\DATA_19.png", 0);
	image[95] = imread("Tram4\\COBOT\\DATA_20.png", 0);
	image[96] = imread("Tram4\\COBOT\\DATA_21.png", 0);
	image[97] = imread("Tram4\\COBOT\\DATA_22.png", 0);
	image[98] = imread("Tram4\\COBOT\\DATA_23.png", 0);
	image[99] = imread("Tram4\\COBOT\\DATA_24.png", 0);
	image[100] = imread("Tram1\\KBOT\\DATA_0.png", 0);
	image[101] = imread("Tram1\\KBOT\\DATA_1.png", 0);
	image[102] = imread("Tram1\\KBOT\\DATA_2.png", 0);
	image[103] = imread("Tram1\\KBOT\\DATA_3.png", 0);
	image[104] = imread("Tram1\\KBOT\\DATA_4.png", 0);
	image[105] = imread("Tram1\\KBOT\\DATA_5.png", 0);
	image[106] = imread("Tram1\\KBOT\\DATA_6.png", 0);
	image[107] = imread("Tram1\\KBOT\\DATA_7.png", 0);
	image[108] = imread("Tram1\\KBOT\\DATA_8.png", 0);
	image[109] = imread("Tram1\\KBOT\\DATA_9.png", 0);
	image[110] = imread("Tram1\\KBOT\\DATA_10.png", 0);
	image[111] = imread("Tram1\\KBOT\\DATA_11.png", 0);
	image[112] = imread("Tram1\\KBOT\\DATA_12.png", 0);
	image[113] = imread("Tram1\\KBOT\\DATA_13.png", 0);
	image[114] = imread("Tram1\\KBOT\\DATA_14.png", 0);
	image[115] = imread("Tram1\\KBOT\\DATA_15.png", 0);
	image[116] = imread("Tram1\\KBOT\\DATA_16.png", 0);
	image[117] = imread("Tram1\\KBOT\\DATA_17.png", 0);
	image[118] = imread("Tram1\\KBOT\\DATA_18.png", 0);
	image[119] = imread("Tram1\\KBOT\\DATA_19.png", 0);
	image[120] = imread("Tram1\\KBOT\\DATA_20.png", 0);
	image[121] = imread("Tram1\\KBOT\\DATA_21.png", 0);
	image[122] = imread("Tram1\\KBOT\\DATA_22.png", 0);
	image[123] = imread("Tram1\\KBOT\\DATA_23.png", 0);
	image[124] = imread("Tram1\\KBOT\\DATA_24.png", 0);
	image[125] = imread("Tram2\\KBOT\\DATA_0.png", 0);
	image[126] = imread("Tram2\\KBOT\\DATA_1.png", 0);
	image[127] = imread("Tram2\\KBOT\\DATA_2.png", 0);
	image[128] = imread("Tram2\\KBOT\\DATA_3.png", 0);
	image[129] = imread("Tram2\\KBOT\\DATA_4.png", 0);
	image[130] = imread("Tram2\\KBOT\\DATA_5.png", 0);
	image[131] = imread("Tram2\\KBOT\\DATA_6.png", 0);
	image[132] = imread("Tram2\\KBOT\\DATA_7.png", 0);
	image[133] = imread("Tram2\\KBOT\\DATA_8.png", 0);
	image[134] = imread("Tram2\\KBOT\\DATA_9.png", 0);
	image[135] = imread("Tram2\\KBOT\\DATA_10.png", 0);
	image[136] = imread("Tram2\\KBOT\\DATA_11.png", 0);
	image[137] = imread("Tram2\\KBOT\\DATA_12.png", 0);
	image[138] = imread("Tram2\\KBOT\\DATA_13.png", 0);
	image[139] = imread("Tram2\\KBOT\\DATA_14.png", 0);
	image[140] = imread("Tram2\\KBOT\\DATA_15.png", 0);
	image[141] = imread("Tram2\\KBOT\\DATA_16.png", 0);
	image[142] = imread("Tram2\\KBOT\\DATA_17.png", 0);
	image[143] = imread("Tram2\\KBOT\\DATA_18.png", 0);
	image[144] = imread("Tram2\\KBOT\\DATA_19.png", 0);
	image[145] = imread("Tram2\\KBOT\\DATA_20.png", 0);
	image[146] = imread("Tram2\\KBOT\\DATA_21.png", 0);
	image[147] = imread("Tram2\\KBOT\\DATA_22.png", 0);
	image[148] = imread("Tram2\\KBOT\\DATA_23.png", 0);
	image[149] = imread("Tram2\\KBOT\\DATA_24.png", 0);
	image[150] = imread("Tram3\\KBOT\\DATA_0.png", 0);
	image[151] = imread("Tram3\\KBOT\\DATA_1.png", 0);
	image[152] = imread("Tram3\\KBOT\\DATA_2.png", 0);
	image[153] = imread("Tram3\\KBOT\\DATA_3.png", 0);
	image[154] = imread("Tram3\\KBOT\\DATA_4.png", 0);
	image[155] = imread("Tram3\\KBOT\\DATA_5.png", 0);
	image[156] = imread("Tram3\\KBOT\\DATA_6.png", 0);
	image[157] = imread("Tram3\\KBOT\\DATA_7.png", 0);
	image[158] = imread("Tram3\\KBOT\\DATA_8.png", 0);
	image[159] = imread("Tram3\\KBOT\\DATA_9.png", 0);
	image[160] = imread("Tram3\\KBOT\\DATA_10.png", 0);
	image[161] = imread("Tram3\\KBOT\\DATA_11.png", 0);
	image[162] = imread("Tram3\\KBOT\\DATA_12.png", 0);
	image[163] = imread("Tram3\\KBOT\\DATA_13.png", 0);
	image[164] = imread("Tram3\\KBOT\\DATA_14.png", 0);
	image[165] = imread("Tram3\\KBOT\\DATA_15.png", 0);
	image[166] = imread("Tram3\\KBOT\\DATA_16.png", 0);
	image[167] = imread("Tram3\\KBOT\\DATA_17.png", 0);
	image[168] = imread("Tram3\\KBOT\\DATA_18.png", 0);
	image[169] = imread("Tram3\\KBOT\\DATA_19.png", 0);
	image[170] = imread("Tram3\\KBOT\\DATA_20.png", 0);
	image[171] = imread("Tram3\\KBOT\\DATA_21.png", 0);
	image[172] = imread("Tram3\\KBOT\\DATA_22.png", 0);
	image[173] = imread("Tram3\\KBOT\\DATA_23.png", 0);
	image[174] = imread("Tram3\\KBOT\\DATA_24.png", 0);
	image[175] = imread("Tram4\\KBOT\\DATA_0.png", 0);
	image[176] = imread("Tram4\\KBOT\\DATA_1.png", 0);
	image[177] = imread("Tram4\\KBOT\\DATA_2.png", 0);
	image[178] = imread("Tram4\\KBOT\\DATA_3.png", 0);
	image[179] = imread("Tram4\\KBOT\\DATA_4.png", 0);
	image[180] = imread("Tram4\\KBOT\\DATA_5.png", 0);
	image[181] = imread("Tram4\\KBOT\\DATA_6.png", 0);
	image[182] = imread("Tram4\\KBOT\\DATA_7.png", 0);
	image[183] = imread("Tram4\\KBOT\\DATA_8.png", 0);
	image[184] = imread("Tram4\\KBOT\\DATA_9.png", 0);
	image[185] = imread("Tram4\\KBOT\\DATA_10.png", 0);
	image[186] = imread("Tram4\\KBOT\\DATA_11.png", 0);
	image[187] = imread("Tram4\\KBOT\\DATA_12.png", 0);
	image[188] = imread("Tram4\\KBOT\\DATA_13.png", 0);
	image[189] = imread("Tram4\\KBOT\\DATA_14.png", 0);
	image[190] = imread("Tram4\\KBOT\\DATA_15.png", 0);
	image[191] = imread("Tram4\\KBOT\\DATA_16.png", 0);
	image[192] = imread("Tram4\\KBOT\\DATA_17.png", 0);
	image[193] = imread("Tram4\\KBOT\\DATA_18.png", 0);
	image[194] = imread("Tram4\\KBOT\\DATA_19.png", 0);
	image[195] = imread("Tram4\\KBOT\\DATA_20.png", 0);
	image[196] = imread("Tram4\\KBOT\\DATA_21.png", 0);
	image[197] = imread("Tram4\\KBOT\\DATA_22.png", 0);
	image[198] = imread("Tram4\\KBOT\\DATA_23.png", 0);
	image[199] = imread("Tram4\\KBOT\\DATA_24.png", 0);
	resize(image[0], image[0], Size(width, height));
	resize(image[1], image[1], Size(width, height));
	resize(image[2], image[2], Size(width, height));
	resize(image[3], image[3], Size(width, height));
	resize(image[4], image[4], Size(width, height));
	resize(image[5], image[5], Size(width, height));
	resize(image[6], image[6], Size(width, height));
	resize(image[7], image[7], Size(width, height));
	resize(image[8], image[8], Size(width, height));
	resize(image[9], image[9], Size(width, height));
	resize(image[10], image[10], Size(width, height));
	resize(image[11], image[11], Size(width, height));
	resize(image[12], image[12], Size(width, height));
	resize(image[13], image[13], Size(width, height));
	resize(image[14], image[14], Size(width, height));
	resize(image[15], image[15], Size(width, height));
	resize(image[16], image[16], Size(width, height));
	resize(image[17], image[17], Size(width, height));
	resize(image[18], image[18], Size(width, height));
	resize(image[19], image[19], Size(width, height));
	resize(image[20], image[20], Size(width, height));
	resize(image[21], image[21], Size(width, height));
	resize(image[22], image[22], Size(width, height));
	resize(image[23], image[23], Size(width, height));
	resize(image[24], image[24], Size(width, height));
	resize(image[25], image[25], Size(width, height));
	resize(image[26], image[26], Size(width, height));
	resize(image[27], image[27], Size(width, height));
	resize(image[28], image[28], Size(width, height));
	resize(image[29], image[29], Size(width, height));
	resize(image[30], image[30], Size(width, height));
	resize(image[31], image[31], Size(width, height));
	resize(image[32], image[32], Size(width, height));
	resize(image[33], image[33], Size(width, height));
	resize(image[34], image[34], Size(width, height));
	resize(image[35], image[35], Size(width, height));
	resize(image[36], image[36], Size(width, height));
	resize(image[37], image[37], Size(width, height));
	resize(image[38], image[38], Size(width, height));
	resize(image[39], image[39], Size(width, height));
	resize(image[40], image[40], Size(width, height));
	resize(image[41], image[41], Size(width, height));
	resize(image[42], image[42], Size(width, height));
	resize(image[43], image[43], Size(width, height));
	resize(image[44], image[44], Size(width, height));
	resize(image[45], image[45], Size(width, height));
	resize(image[46], image[46], Size(width, height));
	resize(image[47], image[47], Size(width, height));
	resize(image[48], image[48], Size(width, height));
	resize(image[49], image[49], Size(width, height));
	resize(image[50], image[50], Size(width, height));
	resize(image[51], image[51], Size(width, height));
	resize(image[52], image[52], Size(width, height));
	resize(image[53], image[53], Size(width, height));
	resize(image[54], image[54], Size(width, height));
	resize(image[55], image[55], Size(width, height));
	resize(image[56], image[56], Size(width, height));
	resize(image[57], image[57], Size(width, height));
	resize(image[58], image[58], Size(width, height));
	resize(image[59], image[59], Size(width, height));
	resize(image[60], image[60], Size(width, height));
	resize(image[61], image[61], Size(width, height));
	resize(image[62], image[62], Size(width, height));
	resize(image[63], image[63], Size(width, height));
	resize(image[64], image[64], Size(width, height));
	resize(image[65], image[65], Size(width, height));
	resize(image[66], image[66], Size(width, height));
	resize(image[67], image[67], Size(width, height));
	resize(image[68], image[68], Size(width, height));
	resize(image[69], image[69], Size(width, height));
	resize(image[70], image[70], Size(width, height));
	resize(image[71], image[71], Size(width, height));
	resize(image[72], image[72], Size(width, height));
	resize(image[73], image[73], Size(width, height));
	resize(image[74], image[74], Size(width, height));
	resize(image[75], image[75], Size(width, height));
	resize(image[76], image[76], Size(width, height));
	resize(image[77], image[77], Size(width, height));
	resize(image[78], image[78], Size(width, height));
	resize(image[79], image[79], Size(width, height));
	resize(image[80], image[80], Size(width, height));
	resize(image[81], image[81], Size(width, height));
	resize(image[82], image[82], Size(width, height));
	resize(image[83], image[83], Size(width, height));
	resize(image[84], image[84], Size(width, height));
	resize(image[85], image[85], Size(width, height));
	resize(image[86], image[86], Size(width, height));
	resize(image[87], image[87], Size(width, height));
	resize(image[88], image[88], Size(width, height));
	resize(image[89], image[89], Size(width, height));
	resize(image[90], image[90], Size(width, height));
	resize(image[91], image[91], Size(width, height));
	resize(image[92], image[92], Size(width, height));
	resize(image[93], image[93], Size(width, height));
	resize(image[94], image[94], Size(width, height));
	resize(image[95], image[95], Size(width, height));
	resize(image[96], image[96], Size(width, height));
	resize(image[97], image[97], Size(width, height));
	resize(image[98], image[98], Size(width, height));
	resize(image[99], image[99], Size(width, height));
	resize(image[100], image[100], Size(width, height));
	resize(image[101], image[101], Size(width, height));
	resize(image[102], image[102], Size(width, height));
	resize(image[103], image[103], Size(width, height));
	resize(image[104], image[104], Size(width, height));
	resize(image[105], image[105], Size(width, height));
	resize(image[106], image[106], Size(width, height));
	resize(image[107], image[107], Size(width, height));
	resize(image[108], image[108], Size(width, height));
	resize(image[109], image[109], Size(width, height));
	resize(image[110], image[110], Size(width, height));
	resize(image[111], image[111], Size(width, height));
	resize(image[112], image[112], Size(width, height));
	resize(image[113], image[113], Size(width, height));
	resize(image[114], image[114], Size(width, height));
	resize(image[115], image[115], Size(width, height));
	resize(image[116], image[116], Size(width, height));
	resize(image[117], image[117], Size(width, height));
	resize(image[118], image[118], Size(width, height));
	resize(image[119], image[119], Size(width, height));
	resize(image[120], image[120], Size(width, height));
	resize(image[121], image[121], Size(width, height));
	resize(image[122], image[122], Size(width, height));
	resize(image[123], image[123], Size(width, height));
	resize(image[124], image[124], Size(width, height));
	resize(image[125], image[125], Size(width, height));
	resize(image[126], image[126], Size(width, height));
	resize(image[127], image[127], Size(width, height));
	resize(image[128], image[128], Size(width, height));
	resize(image[129], image[129], Size(width, height));
	resize(image[130], image[130], Size(width, height));
	resize(image[131], image[131], Size(width, height));
	resize(image[132], image[132], Size(width, height));
	resize(image[133], image[133], Size(width, height));
	resize(image[134], image[134], Size(width, height));
	resize(image[135], image[135], Size(width, height));
	resize(image[136], image[136], Size(width, height));
	resize(image[137], image[137], Size(width, height));
	resize(image[138], image[138], Size(width, height));
	resize(image[139], image[139], Size(width, height));
	resize(image[140], image[140], Size(width, height));
	resize(image[141], image[141], Size(width, height));
	resize(image[142], image[142], Size(width, height));
	resize(image[143], image[143], Size(width, height));
	resize(image[144], image[144], Size(width, height));
	resize(image[145], image[145], Size(width, height));
	resize(image[146], image[146], Size(width, height));
	resize(image[147], image[147], Size(width, height));
	resize(image[148], image[148], Size(width, height));
	resize(image[149], image[149], Size(width, height));
	resize(image[150], image[150], Size(width, height));
	resize(image[151], image[151], Size(width, height));
	resize(image[152], image[152], Size(width, height));
	resize(image[153], image[153], Size(width, height));
	resize(image[154], image[154], Size(width, height));
	resize(image[155], image[155], Size(width, height));
	resize(image[156], image[156], Size(width, height));
	resize(image[157], image[157], Size(width, height));
	resize(image[158], image[158], Size(width, height));
	resize(image[159], image[159], Size(width, height));
	resize(image[160], image[160], Size(width, height));
	resize(image[161], image[161], Size(width, height));
	resize(image[162], image[162], Size(width, height));
	resize(image[163], image[163], Size(width, height));
	resize(image[164], image[164], Size(width, height));
	resize(image[165], image[165], Size(width, height));
	resize(image[166], image[166], Size(width, height));
	resize(image[167], image[167], Size(width, height));
	resize(image[168], image[168], Size(width, height));
	resize(image[169], image[169], Size(width, height));
	resize(image[170], image[170], Size(width, height));
	resize(image[171], image[171], Size(width, height));
	resize(image[172], image[172], Size(width, height));
	resize(image[173], image[173], Size(width, height));
	resize(image[174], image[174], Size(width, height));
	resize(image[175], image[175], Size(width, height));
	resize(image[176], image[176], Size(width, height));
	resize(image[177], image[177], Size(width, height));
	resize(image[178], image[178], Size(width, height));
	resize(image[179], image[179], Size(width, height));
	resize(image[180], image[180], Size(width, height));
	resize(image[181], image[181], Size(width, height));
	resize(image[182], image[182], Size(width, height));
	resize(image[183], image[183], Size(width, height));
	resize(image[184], image[184], Size(width, height));
	resize(image[185], image[185], Size(width, height));
	resize(image[186], image[186], Size(width, height));
	resize(image[187], image[187], Size(width, height));
	resize(image[188], image[188], Size(width, height));
	resize(image[189], image[189], Size(width, height));
	resize(image[190], image[190], Size(width, height));
	resize(image[191], image[191], Size(width, height));
	resize(image[192], image[192], Size(width, height));
	resize(image[193], image[193], Size(width, height));
	resize(image[194], image[194], Size(width, height));
	resize(image[195], image[195], Size(width, height));
	resize(image[196], image[196], Size(width, height));
	resize(image[197], image[197], Size(width, height));
	resize(image[198], image[198], Size(width, height));
	resize(image[199], image[199], Size(width, height));


	Mat new_image(200, height*width, CV_32FC1); //Training sample from input images
	int ii = 0;
	for (int i = 0; i < num_files; i++) {
		Mat temp = image[i];
		ii = 0;
		for (int j = 0; j < temp.rows; j++) {
			for (int k = 0; k < temp.cols; k++) {
				new_image.at<float>(i, ii++) = temp.at<uchar>(j, k);
			}
		}
	}
	//new_image.push_back(image[0].reshape(0, 1));
	//new_image.push_back(image[1].reshape(0, 1));
	Mat labels(num_files, 1, CV_32FC1);
	labels.at<float>(0, 0) = 1.0;
	labels.at<float>(1, 0) = 1.0;
	labels.at<float>(2, 0) = 1.0;
	labels.at<float>(3, 0) = 1.0;
	labels.at<float>(4, 0) = 1.0;
	labels.at<float>(5, 0) = 1.0;
	labels.at<float>(6, 0) = 1.0;
	labels.at<float>(7, 0) = 1.0;
	labels.at<float>(8, 0) = 1.0;
	labels.at<float>(9, 0) = 1.0;
	labels.at<float>(10, 0) = 1.0;
	labels.at<float>(11, 0) = 1.0;
	labels.at<float>(12, 0) = 1.0;
	labels.at<float>(13, 0) = 1.0;
	labels.at<float>(14, 0) = 1.0;
	labels.at<float>(15, 0) = 1.0;
	labels.at<float>(16, 0) = 1.0;
	labels.at<float>(17, 0) = 1.0;
	labels.at<float>(18, 0) = 1.0;
	labels.at<float>(19, 0) = 1.0;
	labels.at<float>(20, 0) = 1.0;
	labels.at<float>(21, 0) = 1.0;
	labels.at<float>(22, 0) = 1.0;
	labels.at<float>(23, 0) = 1.0;
	labels.at<float>(24, 0) = 1.0;
	labels.at<float>(25, 0) = 1.0;
	labels.at<float>(26, 0) = 1.0;
	labels.at<float>(27, 0) = 1.0;
	labels.at<float>(28, 0) = 1.0;
	labels.at<float>(29, 0) = 1.0;
	labels.at<float>(30, 0) = 1.0;
	labels.at<float>(31, 0) = 1.0;
	labels.at<float>(32, 0) = 1.0;
	labels.at<float>(33, 0) = 1.0;
	labels.at<float>(34, 0) = 1.0;
	labels.at<float>(35, 0) = 1.0;
	labels.at<float>(36, 0) = 1.0;
	labels.at<float>(37, 0) = 1.0;
	labels.at<float>(38, 0) = 1.0;
	labels.at<float>(39, 0) = 1.0;
	labels.at<float>(40, 0) = 1.0;
	labels.at<float>(41, 0) = 1.0;
	labels.at<float>(42, 0) = 1.0;
	labels.at<float>(43, 0) = 1.0;
	labels.at<float>(44, 0) = 1.0;
	labels.at<float>(45, 0) = 1.0;
	labels.at<float>(46, 0) = 1.0;
	labels.at<float>(47, 0) = 1.0;
	labels.at<float>(48, 0) = 1.0;
	labels.at<float>(49, 0) = 1.0;
	labels.at<float>(50, 0) = 1.0;
	labels.at<float>(51, 0) = 1.0;
	labels.at<float>(52, 0) = 1.0;
	labels.at<float>(53, 0) = 1.0;
	labels.at<float>(54, 0) = 1.0;
	labels.at<float>(55, 0) = 1.0;
	labels.at<float>(56, 0) = 1.0;
	labels.at<float>(57, 0) = 1.0;
	labels.at<float>(58, 0) = 1.0;
	labels.at<float>(59, 0) = 1.0;
	labels.at<float>(60, 0) = 1.0;
	labels.at<float>(61, 0) = 1.0;
	labels.at<float>(62, 0) = 1.0;
	labels.at<float>(63, 0) = 1.0;
	labels.at<float>(64, 0) = 1.0;
	labels.at<float>(65, 0) = 1.0;
	labels.at<float>(66, 0) = 1.0;
	labels.at<float>(67, 0) = 1.0;
	labels.at<float>(68, 0) = 1.0;
	labels.at<float>(69, 0) = 1.0;
	labels.at<float>(70, 0) = 1.0;
	labels.at<float>(71, 0) = 1.0;
	labels.at<float>(72, 0) = 1.0;
	labels.at<float>(73, 0) = 1.0;
	labels.at<float>(74, 0) = 1.0;
	labels.at<float>(75, 0) = 1.0;
	labels.at<float>(76, 0) = 1.0;
	labels.at<float>(77, 0) = 1.0;
	labels.at<float>(78, 0) = 1.0;
	labels.at<float>(79, 0) = 1.0;
	labels.at<float>(80, 0) = 1.0;
	labels.at<float>(81, 0) = 1.0;
	labels.at<float>(82, 0) = 1.0;
	labels.at<float>(83, 0) = 1.0;
	labels.at<float>(84, 0) = 1.0;
	labels.at<float>(85, 0) = 1.0;
	labels.at<float>(86, 0) = 1.0;
	labels.at<float>(87, 0) = 1.0;
	labels.at<float>(88, 0) = 1.0;
	labels.at<float>(89, 0) = 1.0;
	labels.at<float>(90, 0) = 1.0;
	labels.at<float>(91, 0) = 1.0;
	labels.at<float>(92, 0) = 1.0;
	labels.at<float>(93, 0) = 1.0;
	labels.at<float>(94, 0) = 1.0;
	labels.at<float>(95, 0) = 1.0;
	labels.at<float>(96, 0) = 1.0;
	labels.at<float>(97, 0) = 1.0;
	labels.at<float>(98, 0) = 1.0;
	labels.at<float>(99, 0) = 1.0;
	labels.at<float>(100, 0) = -1.0;
	labels.at<float>(101, 0) = -1.0;
	labels.at<float>(102, 0) = -1.0;
	labels.at<float>(103, 0) = -1.0;
	labels.at<float>(104, 0) = -1.0;
	labels.at<float>(105, 0) = -1.0;
	labels.at<float>(106, 0) = -1.0;
	labels.at<float>(107, 0) = -1.0;
	labels.at<float>(108, 0) = -1.0;
	labels.at<float>(109, 0) = -1.0;
	labels.at<float>(110, 0) = -1.0;
	labels.at<float>(111, 0) = -1.0;
	labels.at<float>(112, 0) = -1.0;
	labels.at<float>(113, 0) = -1.0;
	labels.at<float>(114, 0) = -1.0;
	labels.at<float>(115, 0) = -1.0;
	labels.at<float>(116, 0) = -1.0;
	labels.at<float>(117, 0) = -1.0;
	labels.at<float>(118, 0) = -1.0;
	labels.at<float>(119, 0) = -1.0;
	labels.at<float>(120, 0) = -1.0;
	labels.at<float>(121, 0) = -1.0;
	labels.at<float>(122, 0) = -1.0;
	labels.at<float>(123, 0) = -1.0;
	labels.at<float>(124, 0) = -1.0;
	labels.at<float>(125, 0) = -1.0;
	labels.at<float>(126, 0) = -1.0;
	labels.at<float>(127, 0) = -1.0;
	labels.at<float>(128, 0) = -1.0;
	labels.at<float>(129, 0) = -1.0;
	labels.at<float>(130, 0) = -1.0;
	labels.at<float>(131, 0) = -1.0;
	labels.at<float>(132, 0) = -1.0;
	labels.at<float>(133, 0) = -1.0;
	labels.at<float>(134, 0) = -1.0;
	labels.at<float>(135, 0) = -1.0;
	labels.at<float>(136, 0) = -1.0;
	labels.at<float>(137, 0) = -1.0;
	labels.at<float>(138, 0) = -1.0;
	labels.at<float>(139, 0) = -1.0;
	labels.at<float>(140, 0) = -1.0;
	labels.at<float>(141, 0) = -1.0;
	labels.at<float>(142, 0) = -1.0;
	labels.at<float>(143, 0) = -1.0;
	labels.at<float>(144, 0) = -1.0;
	labels.at<float>(145, 0) = -1.0;
	labels.at<float>(146, 0) = -1.0;
	labels.at<float>(147, 0) = -1.0;
	labels.at<float>(148, 0) = -1.0;
	labels.at<float>(149, 0) = -1.0;
	labels.at<float>(150, 0) = -1.0;
	labels.at<float>(151, 0) = -1.0;
	labels.at<float>(152, 0) = -1.0;
	labels.at<float>(153, 0) = -1.0;
	labels.at<float>(154, 0) = -1.0;
	labels.at<float>(155, 0) = -1.0;
	labels.at<float>(156, 0) = -1.0;
	labels.at<float>(157, 0) = -1.0;
	labels.at<float>(158, 0) = -1.0;
	labels.at<float>(159, 0) = -1.0;
	labels.at<float>(160, 0) = -1.0;
	labels.at<float>(161, 0) = -1.0;
	labels.at<float>(162, 0) = -1.0;
	labels.at<float>(163, 0) = -1.0;
	labels.at<float>(164, 0) = -1.0;
	labels.at<float>(165, 0) = -1.0;
	labels.at<float>(166, 0) = -1.0;
	labels.at<float>(167, 0) = -1.0;
	labels.at<float>(168, 0) = -1.0;
	labels.at<float>(169, 0) = -1.0;
	labels.at<float>(170, 0) = -1.0;
	labels.at<float>(171, 0) = -1.0;
	labels.at<float>(172, 0) = -1.0;
	labels.at<float>(173, 0) = -1.0;
	labels.at<float>(174, 0) = -1.0;
	labels.at<float>(175, 0) = -1.0;
	labels.at<float>(176, 0) = -1.0;
	labels.at<float>(177, 0) = -1.0;
	labels.at<float>(178, 0) = -1.0;
	labels.at<float>(179, 0) = -1.0;
	labels.at<float>(180, 0) = -1.0;
	labels.at<float>(181, 0) = -1.0;
	labels.at<float>(182, 0) = -1.0;
	labels.at<float>(183, 0) = -1.0;
	labels.at<float>(184, 0) = -1.0;
	labels.at<float>(185, 0) = -1.0;
	labels.at<float>(186, 0) = -1.0;
	labels.at<float>(187, 0) = -1.0;
	labels.at<float>(188, 0) = -1.0;
	labels.at<float>(189, 0) = -1.0;
	labels.at<float>(190, 0) = -1.0;
	labels.at<float>(191, 0) = -1.0;
	labels.at<float>(192, 0) = -1.0;
	labels.at<float>(193, 0) = -1.0;
	labels.at<float>(194, 0) = -1.0;
	labels.at<float>(195, 0) = -1.0;
	labels.at<float>(196, 0) = -1.0;
	labels.at<float>(197, 0) = -1.0;
	labels.at<float>(198, 0) = -1.0;
	labels.at<float>(199, 0) = -1.0;

	printf("%f %f", labels.at<float>(0, 0), labels.at<float>(1, 0));
	CvSVMParams params;
	params.svm_type = CvSVM::C_SVC;
	params.kernel_type = CvSVM::LINEAR;
	params.gamma = 3;
	params.degree = 3;
	params.term_crit = cvTermCriteria(CV_TERMCRIT_ITER, 200, 1e-16);
	CvSVM svm;
	svm.train(new_image, labels, Mat(), Mat(), params);
	svm.save("svm.xml"); // saving
	svm.load("svm.xml"); // loading
						// VideoCapture cap(0);//code load camera	 
	VideoCapture cap1("clip1.avi");// doi duong dan den file can test
	VideoCapture cap2("clip2.avi");// doi duong dan den file can test
	VideoCapture cap3("clip3.avi");// doi duong dan den file can test
	VideoCapture cap4("clip4.avi");// doi duong dan den file can test
	ifstream indata;
	ofstream outdata;
	outdata.open("ketQua.csv", ios::app);

	outdata << "Ngay,Trang thai, CAM " << endl;
	int t = 0;
	while (1)
	{
		t++;
		cap1.read(frame);

		if (frame.empty()) break;
		cvtColor(frame, edges, CV_BGR2GRAY);		
		Canny(edges, dst1, 80, 120, 3, false);		
		
		namedWindow("Tram 1", WINDOW_NORMAL);		
		moveWindow("Tram 1", 0, 40);
		imshow("Tram 1", frame);

		cap2.read(frame);
		cvtColor(frame, edges2, CV_BGR2GRAY);
		Canny(edges2, dst2, 80, 120, 3, false);
		namedWindow("Tram 2", WINDOW_NORMAL);
		moveWindow("Tram 2", 330, 40);
		imshow("Tram 2", frame);
		

		cap3.read(frame);
		cvtColor(frame, edges3, CV_BGR2GRAY);
		Canny(edges3, dst3, 80, 120, 3, false);
		
		namedWindow("Tram3", WINDOW_NORMAL);
		moveWindow("Tram3", 660, 40);
		imshow("Tram3", frame);
		

		cap4.read(frame);
		cvtColor(frame, edges4, CV_BGR2GRAY);
		Canny(edges4, dst4, 80, 120, 3, false);
		namedWindow("Tram 4", WINDOW_NORMAL);
		moveWindow("Tram 4", 990, 40);
		imshow("Tram 4", frame);
		


		resize(dst1, dst1, Size(width, height));
		dst1 = dst1.reshape(0, 1);
		dst1.convertTo(dst1, CV_32FC1);

		resize(dst2, dst2, Size(width, height));
		dst2 = dst2.reshape(0, 1);
		dst2.convertTo(dst2, CV_32FC1);

		resize(dst3, dst3, Size(width, height));
		dst3 = dst3.reshape(0, 1);
		dst3.convertTo(dst3, CV_32FC1);

		resize(dst4, dst4, Size(width, height));
		dst4 = dst4.reshape(0, 1);
		dst4.convertTo(dst4, CV_32FC1);


		float res = svm.predict(dst1);
		float res2 = svm.predict(dst2);
		float res3 = svm.predict(dst3);
		float res4 = svm.predict(dst4);
		if (res > 0) 
			{		
			
			SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), 8);
				cout << endl << "Tram1: Co bot khi ";
			}
		else {
			
			SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), 4);
			cout << endl << "Tram1: Khong co bot khi";
			Beep(4500, 10);
		}
	if (res2 > 0) {
		SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), 8);
			cout << endl << "					Tram2: Co bot khi ";
		}
		else 
		{
						
			SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), 4);
			cout << endl << "					Tram2: khong co bot khi";
			Beep(4500, 10);
		}
		
		
	if (res3 > 0) {
		SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), 8);
		cout << endl << "										Tram3: Co bot khi ";
	}
	else
	{

		SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), 4);
		cout << endl << "										Tram3: khong co bot khi";
		Beep(4500, 10);
	}
	if (res4 > 0) {
		SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), 8);
		cout << endl << "															Tram4:  Co bot khi ";
		
	}
	else 
	{

		SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), 4);
		cout << endl << "															Tram4: khong co bot khi";
		Beep(4500, 10);
	}


	time_t hientai = time(0);

	// chuyen doi hientai thanh dang chuoi
	char* dt = ctime(&hientai);

	// chuyen doi hientai thanh dang tm struct cho UTC
	tm *gmtm = gmtime(&hientai);
	dt = asctime(gmtm);
	
	if(t == 240)
	{
		if (res < 0)
		{
			outdata << dt << ",khong co  bot , Tram 1" << endl;
		}
		if (res2 < 0)
		{
			outdata << dt << ",khong co  bot , Tram 2" << endl;
		}
		if (res3 < 0)
		{
			outdata << dt << ",khong co  bot  , Tram 3" << endl;
		}
		if (res4 < 0)
		{
			outdata << dt << ",khong co bot , Tram 4" << endl;
		}
		indata.open("aaron.csv");
		t = 240 - 240;
	}
	
	
		char key = waitKey(10);
		if (key == 27) break;
		
	}//system("pause");
}







