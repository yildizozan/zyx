#include <stdio.h>
#include <opencv2/opencv.hpp>
using namespace cv;
int main(int argc, char** argv )
{
	if ( argc != 2 )
	{
		printf("usage: DisplayImage.out <Image_Path>\n");
		return -1;
	}
	Mat src, dst;
	src = imread( argv[1], 1 );
	if ( !src.data )
	{
		printf("No image data \n");
		return -1;
	}
	GaussianBlur( src, dst, Size( 3, 3 ), 3, 3 );
	imwrite("result.png", dst);
	return 0;
}
