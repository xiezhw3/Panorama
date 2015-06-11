#include "Panorama.h"
#include "cropper.h"

#include <vector>
#include <string>
#include <iostream>

using namespace std;

int main(int argc, char *argv[])
{
	std::vector<cv::Mat> vecImgs;

	cout << "Loading image..." << endl;
	for (int i = 0; i != 18; ++i) {
		vecImgs.push_back(cv::imread("image/" + std::to_string(i) + ".JPG"));
	}

	cv::Mat panoImg = panorama(vecImgs);

	cout << "Cropping image..." << endl;
	// 去掉图片边缘黑框部分
	Cropper *cropper = new Cropper();
	panoImg = cropper->crop(panoImg);

	cout << "Writting image..." << endl;
	cv::imwrite("result/result.jpg", panoImg);

	cout << "Finish all" << endl;

	return 0;
}