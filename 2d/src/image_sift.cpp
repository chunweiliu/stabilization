// Retrieve SIFT features from an image sequence
// Dependancies: convert (ImageMagicsk), sift (David Lowe)
// Output: SIFT FEATURES @ <output_image_dir>%04d.key
#include <cstdio>
#include <iostream>
#include <cstring>

#include <cv.h>
#include <highgui.h>

#include<unistd.h>
#include<ctime>

static const char SIFT_BIN[] = "../bin/sift";
using namespace std;

void parseFileName( char *input_path, char *image_name, char &image_postfix ) {
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

int main ( int argc, char *argv[] )
{
    if (argc != 4) {
	cerr << "Usage: " << argv[0] 
	     << " <input_image_dir> <tmp_pgm_dir> <output_key_dir>"
	     << endl;
	return -1;
    }

    const char *input_image_dir	= argv[1];
    const char *tmp_dir		= argv[2]; // important for parallel excute this program
    const char *output_key_dir 	= argv[3];

    // Parsing input_image_dir. Eg. ../dat/imagename_l/ => imagename l
    // Find the last dir in the input path, and devide them into name and postfix.
    char image_postfix;
    char image_name[128];
    parseFileName( argv[1], image_name, image_postfix );
    
    // Load images sequencally from input dir, and use sift binary to detect and descripte the features
    IplImage *frame;
    int frame_num = -1;
    srand(time(NULL));
    while (1) {
	++frame_num;
	char image_file[128];
	sprintf( image_file, "%s%s_%04d_%c.png", input_image_dir, image_name, frame_num, image_postfix );
	frame = cvLoadImage( image_file );

	if (frame == NULL) break;
	char output_file[128];
	sprintf( output_file, "%s%s_%04d_%c.key",  output_key_dir, image_name, frame_num, image_postfix );
	if ( access( output_file, F_OK )==0 ) continue;
	// if (frame_num == 1) break; // debug 

	// Generate a tmp dir for sift binary. If you need to parallelly run the program, you have to give
	// different tmp dir name on input.
	IplImage *gray_image = cvCreateImage( cvGetSize(frame), IPL_DEPTH_8U, 1 );
	if (frame->nChannels == 3) {
	    cvCvtColor( frame, gray_image, CV_RGB2GRAY );
	} else if (frame->nChannels == 1) {
	    cvCopy( frame, gray_image );
	} else {
	    return -1;
	}
	char tmp_png_file[128];
	char tmp_pgm_file[128];
	int  rand_num = rand();
	sprintf( tmp_png_file, "%stmp%d.png", tmp_dir, rand_num );
	sprintf( tmp_pgm_file, "%stmp%d.pgm", tmp_dir, rand_num );
	cvSaveImage( tmp_png_file, gray_image );

	char cmd[128];
	sprintf( cmd, "convert %s %s", tmp_png_file, tmp_pgm_file );
	system( cmd );
	sprintf( cmd, "%s < %s >%s", SIFT_BIN, tmp_pgm_file, output_file );
	system( cmd );
	sprintf( cmd, "rm -f %s", tmp_pgm_file );
	system( cmd );
	sprintf( cmd, "rm -f %s", tmp_png_file );
	system( cmd );
	cvReleaseImage( &gray_image );
    }
}
