#include <iostream>
#include <fstream>
#include <vector>

#include <cstdio>
#include <cstring>

#include <cv.h>
#include <highgui.h>

using namespace std;

static void loadTransformMatrices(vector<CvMat*> &tList, const char *filename) {
    ifstream in(filename);
    while (true) {
	double a, b, dx, dy;
	in >> a >> b >> dx >> dy;
	if (in.eof()) break;
	CvMat *mat = cvCreateMat(2, 3, CV_32FC1);
	cvSetIdentity(mat);
	cvSetReal2D(mat, 0, 0, a);
	cvSetReal2D(mat, 1, 1, a);
	cvSetReal2D(mat, 0, 1, -b);
	cvSetReal2D(mat, 1, 0, b);
	cvSetReal2D(mat, 0, 2, dx);
	cvSetReal2D(mat, 1, 2, dy);
	tList.push_back(mat);
    }
    in.close();
}

void parseFileName( const char *input_path, char *image_name, char &image_postfix ) {
    char tmp[128];
    strcpy( tmp, input_path );
    char *str;
    char *pre_str;
    str = strtok( tmp, "/" );
    while (str != NULL) {
	pre_str = str;
	str = strtok( NULL, "/" );		
    }	

    int str_len = strlen(pre_str);
    image_postfix = pre_str[str_len-1];

    strncpy (image_name, pre_str, str_len-2);
    image_name[str_len-2] = '\0';

}

int main(int argc, char *argv[]) {
    if (argc != 4) {
	std::cerr << "Usage: " << argv[0];
	std::cerr << " <input_image_dir> <matrix_filename> <output_dir>";
	std::cerr << std::endl;
	return -1;
    }
    const char *image_dir = argv[1];
    const char *matrix_filename = argv[2];
    const char *output_dir = argv[3];

    char image_postfix;
    char image_name[128];
    parseFileName( argv[1], image_name, image_postfix );

    vector<CvMat*> transformList;
    loadTransformMatrices(transformList, matrix_filename);

    //CvCapture *video = cvCreateFileCapture( video_filename );
    IplImage *frame = NULL;
    int frame_num = -1;

    /*static int FRAME_WIDTH = 
      cvGetCaptureProperty(video, CV_CAP_PROP_FRAME_WIDTH );
      static int FRAME_HEIGHT = 
      cvGetCaptureProperty(video, CV_CAP_PROP_FRAME_HEIGHT );*/

    //while ( (frame = cvQueryFrame(video)) != NULL ) {
    while(1) {

	++frame_num;
	char filename[128];
	sprintf( filename, "%s%s_%04d_%c.png", output_dir, image_name, frame_num, image_postfix );
	if (access( filename, F_OK )==0) continue;

	char image_file[128];
	sprintf(image_file, "%s%s_%04d_%c.png", image_dir, image_name, frame_num, image_postfix);
	frame = cvLoadImage(image_file);
	if (frame == NULL) break;
	
	IplImage *output = cvCloneImage(frame);
	cvWarpAffine( frame, output, transformList[frame_num], CV_INTER_CUBIC+CV_WARP_FILL_OUTLIERS );
	/*
	   cvLine(
	   output,
	   cvPoint(0, FRAME_HEIGHT/2),
	   cvPoint(FRAME_WIDTH-1, FRAME_HEIGHT/2),
	   CV_RGB(255, 0, 0), 1, CV_AA);

	   cvLine(
	   output,
	   cvPoint(FRAME_WIDTH/2, 0)
	   cvPoint(FRAME_WIDTH/2, FRAME_HEIGHT-1),
	   CV_RGB(255, 0, 0), 1, CV_AA);
	 */
	{
	    cvSaveImage( filename, output );
	}
	cvReleaseImage(&output);
    }
    cvReleaseImage( &frame );
    return 0;
}
