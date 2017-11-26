//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/ml/ml.hpp>
//#include <opencv2\imgproc\imgproc.hpp>
//#include <iostream>
//using namespace cv;
//using namespace std;
//Mat frame;
//char kt;
//int i = 0, ii = 0;
//Mat dst1;
//Mat edges;
//int main(int argc, char* argv[])
//{
//	cout << "Import Data \n";
//#pragma region INPUT
//#pragma region INPUT DATA 1
//	do
//	{
//		cout << "vui long nhap du lieu co bot khi \n";
//		cout << "nhan enter de tiep tuc\n";
//	} while (cin.get() != '\n');
//	//VideoCapture cap(0);
//	VideoCapture cap("Tram3\\Tram3_DLCoBot.avi");// doi duong dan den clip mau co bot khi
//	while (i <= 99)
//	{
//		cap.read(frame);
//		if (frame.empty()) break;
//		// imshow("video", frame);
//		cvtColor(frame, edges, CV_BGR2GRAY);
//		Canny(edges, dst1, 100, 120, 3, false);
//		//sobel(edges, dst1, 100, 120, 3); robert, prewitt, sobel, top-hat transform
//		//Sobel(edges, edges,  100, 120, 3, false);
//		imshow("MyVideo", frame);
//		imshow("XAM", edges);
//		imshow("XL", dst1);
//		char filename[80];
//		sprintf(filename, "Tram3\\COBOT\\DATA_%d.png", i);
//		imwrite(filename, dst1);
//		i++;
//		char key = waitKey(10);
//		if (key == 27) break;
//	}i = 0;
//	destroyAllWindows();
//	cout << "\nImport data 1 is Done...\n";
//#pragma endregion
//#pragma region INPUT DATA 2
//	do
//	{
//		cout << "vui long nhap du lieu k co bot khi \n";
//		cout << "nhan enter de tiep tuc\n";
//	} while (cin.get() != '\n');
//	// VideoCapture cap1(0);
//	VideoCapture cap1("Tram3\\Tram3_DLKBot.avi");// doi duong dan den mau k co bot khi
//	while (i <= 99)
//	{
//		cap1.read(frame);
//		if (frame.empty()) break;
//		imshow("video", frame);
//		cvtColor(frame, edges, CV_BGR2GRAY);
//		Canny(edges, dst1, 100, 120, 3, false);
//		imshow("XAM", edges);
//		imshow("XL", dst1);
//		char filename[80];
//		sprintf(filename, "Tram3\\KBOT\\DATA_%d.png", i);
//		imwrite(filename, dst1);
//		i++;
//		char key = waitKey(10);
//		if (key == 27) break;
//	}
//	destroyAllWindows();
//	cout << "\nImport data 2 is Done...\n";
//	system("PAUSE");
//#pragma endregion
//#pragma endregion
//}