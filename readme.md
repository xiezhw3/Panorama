#Image Stitch

####Description
	本项目是将多张图片进行组合，拼接成一张全景图片
	
####FileTree
	.
	├── CMakeLists.txt
	├── image
	│   ├── 0.JPG
	│   ├── 1.JPG
	│   ├── 10.JPG
	│   ├── 11.JPG
	│   ├── 12.JPG
	│   ├── 13.JPG
	│   ├── 14.JPG
	│   ├── 15.JPG
	│   ├── 16.JPG
	│   ├── 17.JPG
	│   ├── 2.JPG
	│   ├── 3.JPG
	│   ├── 4.JPG
	│   ├── 5.JPG
	│   ├── 6.JPG
	│   ├── 7.JPG
	│   ├── 8.JPG
	│   ├── 9.JPG
	├── readme.md
	├── result
	├── run.sh
	└── src
	    ├── Panorama.cpp
	    ├── Panorama.h
	    ├── cropper.cpp
	    ├── cropper.h
	    └── main.cpp	
	
####Run
运行时，首先要确保图片在 `image` 文件夹中，并且要保证图片出于竖直状态，因为程序只能按顺序处理图片，所以在程序里面硬编码了图片的输入，也就是图片必须按从 0.jpg ~ n.jpg 的顺序。如果图片命名不是直接的数字，可以直接修改程序 `main` 函数里面的图片读取部分。

运行命令：

	bash run.sh

注意

-	运行环境是 *inux 系统，并且终端是 bash 环境
-	需要安装 opencv2.4.9或以上
-	需要安装 cmake
-	需要 g++ 4.8.1 及以上或 clang 3.3 及以上以支持 c++11

如果在 win 下，那么执行如下指令：

	cmake .
	make
	./imageSitichExe
	
####Result
	结果在 result 文件夹下，文件名为 result.jpg
	
####Contact
Author: Zhiwang Xie

E-mail: xiezhw3@163.com

Github: www.github.com/xiezhw3