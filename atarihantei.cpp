#include <stdlib.h>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <raspicam/raspicam_cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <wiringPi.h>
#include <math.h>

using namespace cv;

float rectX[4];
float rectY[4];
void swap(int n0, int n1);
float dist(int n, float x, float y);

void setupPin();
void readFile(float *calvRectX, float *calvRectY, double *minm, double *max, int *portNum, int *waitTime, int *playerID);

void send5bit(int n);

Mat generateImage(Mat& frame, int threshold);
int hitObject(Mat& binarizedImage, int atariHantei, int sock, int playerID, int objectNum, sockaddr_in addr);
int checkTooClose(Mat& binarizedImage);

int main(int argc, char **argv) {
	printf("push [a] key to up threshold \n push [s] key to down threshold \n");
	if (argc < 4) {
		printf("please ip idress & use display (answer 0(no) or 1(yes) )? and threshold\n");
		return 1;
	}

	char *gomi;
	float calvRectX[4];
	float calvRectY[4];
	double minm = 100;
	double max = 10000;
	int portNum = 0;
	int waitTime;
	int playerID = 0;
	int threshold = atoi(argv[3]);
	readFile(calvRectX, calvRectY, &minm, &max, &portNum, &waitTime, &playerID);
	wiringPiSetupGpio();
	setupPin();

	/*ここから通信準備*/
	int sock;
	struct sockaddr_in addr;
	int f = 23;
	char buf[256];
	sock = socket(AF_INET, SOCK_DGRAM, 0);
	addr.sin_family = AF_INET;
	addr.sin_port = htons(portNum);
	addr.sin_addr.s_addr = inet_addr(argv[1]);
	/*ここまで通信準備*/
	VideoCapture vcap(0);

	/*カメラの設定*/
	//raspicam::RaspiCam_Cv cap;
	//vcap.set(CV_CAP_PROP_FORMAT, CV_8UC1);
	//vcap.set(CV_CAP_PROP_EXPOSURE, -1);
//	vcap.set(CV_CAP_PROP_FPS, 90);
	/*カメラの設定*/

	cv::Mat frame;
	//if (!cap.open()) {
	//      return -1;
	//}


	if (!vcap.isOpened()) {
		printf("cannot open");

		return -1;
	}
        

	vcap.grab();
	vcap.retrieve(frame);
	//vcap >> frame;
	cv::Mat grayScaleImage = cv::Mat::zeros(frame.rows, frame.cols, CV_8UC1);
	cv::Mat binarizedImage = cv::Mat::zeros(frame.rows, frame.cols, CV_8UC1);

	bool pre = false;
	while (cv::waitKey(1) != 'q') {
		if (digitalRead(4) == 0 && pre == false) {
			pre = true;

			/**********************************************************************************/
			//人の判定。銃のライトを光らせる命令を書く
			vcap.grab();
			vcap.grab();
                        vcap.retrieve(frame);
			binarizedImage = generateImage(frame, threshold);
			if (checkTooClose(binarizedImage)) {
				printf("Too Close\n");
				if (atoi(argv[2]) == 1) {
					imshow("binarized 0", binarizedImage);
				}
				continue;
			}
			//delay(waitTime);
			binarizedImage = generateImage(frame, threshold);
			if (hitObject(binarizedImage, 80, sock, playerID, 0, addr) == 1) {
				if (atoi(argv[2]) == 1) {
					imshow("binarized 0", binarizedImage);
				}
				continue;
			}
			if (atoi(argv[2]) == 1) {
				imshow("binarized 0", binarizedImage);
			}

			//delay(waitTime);
			bool con = false;
                         
			for (int i = 1; i <= 8; i++) {
                   
				if(i>7){
				  i = 17;
				}
				send5bit(i);
				vcap.grab();
				vcap.grab();
                                vcap.retrieve(frame);
				binarizedImage = generateImage(frame, threshold);
		
				if (hitObject(binarizedImage, 80, sock, playerID, i, addr) == 1) {
					if (atoi(argv[2]) == 1) {
						imshow("binarized 1", binarizedImage);
					}
					con = true;
					break;
				}
				if (atoi(argv[2]) == 1) {
					imshow("binarized %d", binarizedImage);
				}
			}
			if (con)continue;
			/**********************************************************************************/


			/**********************************************************************************************
			********************************************************************************************:**


			以下壁の話　

			***********************************************************************************************
			***********************************************************************************************/

			/* ラベリング */
			std::vector<std::vector<cv::Point>> contours;
			std::vector<cv::Vec4i> hierarchy;

                        //xy
vcap >> frame;
                        //continue;
			int regionCounter;
			regionCounter = 0;
			int counter = 0;
			findContours(binarizedImage, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
			for (int i = 0; i < contours.size(); i++) {

				double size = contourArea(contours[i]);

				/* 面積で絞る */
				if (size < minm || size > max) {
					continue;
				}
				/* 重心を求める(Momentバージョン) */
				float centroidX;
				float centroidY;
				cv::Moments moment = cv::moments(contours[i], true);
				centroidX = (float)(moment.m10 / moment.m00);
				centroidY = (float)(moment.m01 / moment.m00);

				rectX[counter] = centroidX;
				rectY[counter] = centroidY;
				counter++;
				if (counter >= 4) { break; }

			}//for
			int i = 0;
			while (1) {
				if (dist(i, 0, 0) > dist(i + 1, 0, 0)) { swap(i, i + 1);i = 0; continue; }

				i++;
				if (i == 3) { break; }

			}
			if (dist(3, frame.cols, frame.rows)>dist(1, frame.cols, frame.rows)) { swap(1, 3); }
			if (dist(3, frame.cols, frame.rows)>dist(2, frame.cols, frame.rows)) { swap(2, 3); }
			if (rectX[1]>rectX[2]) { swap(1, 2); }


			for (int i = 0; i < 4; i++) {
				printf("x%d: %d  ", i, (int)(rectX[i]));
				printf("y%d: %d   ", i, (int)(rectY[i]));
			}
			printf("\n");
			line(binarizedImage, Point(rectX[0], rectY[0]), Point(rectX[1], rectY[1]), Scalar(255, 255, 255), 3);
			line(binarizedImage, Point(rectX[1], rectY[1]), Point(rectX[3], rectY[3]), Scalar(255, 255, 255), 3);
			line(binarizedImage, Point(rectX[3], rectY[3]), Point(rectX[2], rectY[2]), Scalar(255, 255, 255), 3);
			line(binarizedImage, Point(rectX[2], rectY[2]), Point(rectX[0], rectY[0]), Scalar(255, 255, 255), 3);
			line(binarizedImage, Point(frame.cols / 2 - 10, frame.rows / 2), Point(frame.cols / 2 + 10, frame.rows / 2), Scalar(255, 255, 255), 3);
			line(binarizedImage, Point(frame.cols / 2, frame.rows / 2 - 10), Point(frame.cols / 2, frame.rows / 2 + 10), Scalar(255, 255, 255), 3);

			Point2f srcPoint[] =
			{
				Point(rectX[0],rectY[0]),
				Point(rectX[1],rectY[1]),
				Point(rectX[2],rectY[2]),
				Point(rectX[3],rectY[3])
			};
			Point2f dstPoint[] =//
			{
				Point(calvRectX[0],calvRectY[0]),
				Point(calvRectX[1],calvRectY[1]),
				Point(calvRectX[2],calvRectY[2]),
				Point(calvRectX[3],calvRectY[3])
			};

			Mat H = getPerspectiveTransform(srcPoint, dstPoint);

			Mat tin(1, 1, CV_32FC2);
			tin.at<Vec2f>(0, 0)[0] = frame.cols / 2;
			tin.at<Vec2f>(0, 0)[1] = frame.rows / 2;
			Mat point(1, 1, CV_32FC2);
			perspectiveTransform(tin, point, H);
			printf("transformedX: %d transformedY: %d\n", (int)(point.at<Vec2f>(0, 0)[0]), (int)(point.at<Vec2f>(0, 0)[1]));
			char tinX[16];char tinY[16];
			snprintf(tinX, 16, "%d", (int)(point.at<Vec2f>(0, 0)[0]));
			snprintf(tinY, 16, "%d", (int)(point.at<Vec2f>(0, 0)[1]));
			putText(binarizedImage, tinX, Point(frame.cols / 2 - 50, frame.rows / 2 - 20.0f), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 1, CV_AA);
			putText(binarizedImage, tinY, Point(frame.cols / 2 + 10, frame.rows / 2 - 20.0f), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 1, CV_AA);

			if ((rectX[0] != 0 && rectY[0] != 0) && 0 <= point.at<Vec2f>(0, 0)[0] && point.at<Vec2f>(0, 0)[0] <= 1920 && 0 <= point.at<Vec2f>(0, 0)[1] && point.at<Vec2f>(0, 0)[1] <= 1080) {

				circle(binarizedImage, Point(frame.cols / 2, frame.rows / 2), 9, Scalar(255, 255, 255), 2, CV_AA, 0);

				f = (int)(point.at<Vec2f>(0, 0)[0]) * 10000 + (int)(point.at<Vec2f>(0, 0)[1]);
				snprintf(buf, sizeof(buf), "%d %d %d", playerID, 0/*objectNumは4点のものにする*/, f);

				printf("HIT!!!! \n\n");
				sendto(sock, buf, sizeof(buf), 0, (struct sockaddr *)&addr, sizeof(addr));
			}
			else {

				printf("failed...\n\n");

			}

			if (atoi(argv[2]) == 1) { cv::imshow("binarized", binarizedImage); }
			for (int i = 0; i<4;i++) {
				rectX[i] = 0;
				rectY[i] = 0;

			}

		}//if*digitalread*4(
		else if (digitalRead(4) == 1) {
			pre = false;
		}

		if (cv::waitKey(1) == 'a') {
			threshold++;
		}
		else if (cv::waitKey(1) == 's') {
			threshold--;
		}

		/**************************************************************************************
		***************************************************************************************

		ここまで壁の話


		***************************************************************************************
		****************************************************************************************/
   
        delay(100);

	}
	return 0;
}

void swap(int n0, int n1) {
	float x, y;
	x = rectX[n0]; y = rectY[n0];
	rectX[n0] = rectX[n1];rectY[n0] = rectY[n1];
	rectX[n1] = x;rectY[n1] = y;
}

float dist(int n, float x, float y) {
	return (rectX[n] - x)*(rectX[n] - x) + (rectY[n] - y)*(rectY[n] - y);
}



void send5bit(int n) {
	digitalWrite(5, 0);

	if (16 <= n) { digitalWrite(11, 1); n = n - 16; }
	else { digitalWrite(11, 0); }
	if (8 <= n) { digitalWrite(26, 1); n = n - 8; }
	else { digitalWrite(26, 0); }
	if (4 <= n) { digitalWrite(19, 1); n = n - 4; }
	else { digitalWrite(19, 0); }
	if (2 <= n) { digitalWrite(13, 1); n = n - 2; }
	else { digitalWrite(13, 0); }
	if (1 == n) { digitalWrite(6, 1); }
	else { digitalWrite(6, 0); }

	digitalWrite(5, 1);
}

void setupPin() {
	pinMode(4, INPUT);
	pullUpDnControl(4, PUD_UP);


	pinMode(5, OUTPUT);
	pinMode(6, OUTPUT);
	pinMode(11, OUTPUT);
	pinMode(13, OUTPUT);
	pinMode(19, OUTPUT);
	pinMode(26, OUTPUT);
	digitalWrite(5, 0);
	digitalWrite(6, 0);
	digitalWrite(11, 0);
	digitalWrite(13, 0);
	digitalWrite(19, 0);
	digitalWrite(26, 0);

}

int hitObject(Mat& binarizedImage, int atariHantei, int sock, int playerID, int objectNum, sockaddr_in addr) {
	char buf[256];
	int tin = 8;

	Mat binarizedImageROI = binarizedImage(Rect((binarizedImage.cols / 2) - 50, (binarizedImage.rows / 2) - 50, 100, 100));

	/* ラベリング */
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	int regionCounter = 0;
	int counter = 0;
	findContours(binarizedImageROI, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	int size = 0;
	for (int i = 0; i < contours.size(); i++) {
		size += (int)contourArea(contours[i]);
	}
	//printf("obj:%d  siro:%lf \n", objectNum,size);
	//printf("obj:%d  ryouiki:%d\n",(int)size / ((int)binarizedImageROI.rows * (int)binarizedImageROI.cols));
	/*sizeが白の領域の面積, 本来の計算式はsize/(binarizedImageROI.cols * binarizedImageROI.rows)に100をかけてパーセンテージを出すんだけど
	ROIの領域が100x100なので簡単にして以下のようにしました*/
	if ((size / 100) > atariHantei) { 
		send5bit(objectNum + tin);
		snprintf(buf, sizeof(buf), "%d %d %d", playerID, objectNum, 0/*4点の座標*/);
		sendto(sock, buf, sizeof(buf), 0, (struct sockaddr *)&addr, sizeof(addr));
		/*白が何パーセントを占めるかプリント*/
		printf("siro: %d per\n", size / 100);
		printf("%d object HIT!!!! \n\n", objectNum);
		return 1;
	}
	else {
		printf("%d NOT HIT\n\n", objectNum);
		return -1;
	}
}

int checkTooClose(Mat& binarizedImage) {
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	int regionCounter = 0;
	int counter = 0;
	findContours(binarizedImage, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	double size = 0;
	for (int i = 0; i < contours.size(); i++) {
		size += contourArea(contours[i]);
	}
	if ((size / (binarizedImage.cols * binarizedImage.rows)) > 0.8) { //80%以上白だったらはじく
		return 1;
	}
	else {
		return 0;
	}
}

void readFile(float *calvRectX, float *calvRectY, double *minm, double *max, int *portNum, int *waitTime, int *playerID) {

	FILE *fp;
	fp = fopen("test.txt", "r");
	if (fp == NULL) {
		printf("ファイルオープンエラー\n");
	}
	printf("ファイルが開けた\n");
	fscanf(fp, "%f %f", &calvRectX[0], &calvRectY[0]);
	fscanf(fp, "%f %f", &calvRectX[1], &calvRectY[1]);
	fscanf(fp, "%f %f", &calvRectX[2], &calvRectY[2]);
	fscanf(fp, "%f %f", &calvRectX[3], &calvRectY[3]);
	fscanf(fp, "%lf %lf", minm, max);
	fscanf(fp, "%d", portNum);
	fscanf(fp, "%d", waitTime);
	fscanf(fp, "%d", playerID);

	printf("%f %f", calvRectX[0], calvRectY[0]);
	printf("%f %f", calvRectX[1], calvRectY[1]);
	printf("%f %f", calvRectX[2], calvRectY[2]);
	printf("%f %f", calvRectX[3], calvRectY[3]);


}

Mat generateImage(Mat& frame, int threshold) {
	/*画像の初期化*/
	Mat grayScaleImage = cv::Mat::zeros(frame.rows, frame.cols, CV_8UC1);
	Mat binarizedImage = cv::Mat::zeros(frame.rows, frame.cols, CV_8UC1);

	/* グレイスケール変換 */
	cv::cvtColor(frame, grayScaleImage, CV_BGR2GRAY);

	/* 2値化 */
	cv::threshold(grayScaleImage, binarizedImage, threshold, 255.0, CV_THRESH_BINARY);
	return binarizedImage;
}
