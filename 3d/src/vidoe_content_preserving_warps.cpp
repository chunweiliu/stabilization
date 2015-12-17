#define _USE_MATH_DEFINES

#include <cstdio>
#include <cstdlib>
#include <vector>
using namespace std;
#include <opencv/highgui.h>
#include <opencv/cv.h>
#include <gl/gl.h>
#include <gl/glut.h>

// the texture width and height must be a power of two 
// ex. for 640x480 images case, use 20x15 grids
#define TEXTURE_WIDTH  512
#define	TEXTURE_HEIGHT 512
//#define GRID_ROW_NUM 15
//#define GRID_COL_NUM 20
//#define GRID_ROW_NUM 30
//#define GRID_COL_NUM 40
// use 5~10 will be fine for our test data 
//#define SMOOTHNESS_ALPHA 7
//#define SMOOTHNESS_ALPHA 20




//char input_image_dir   [128];
//char input_matching_dir[128];
//char output_image_dir  [128];
int  num_frame;
char *input_image_dir;
char *input_matching_dir;
char *output_image_dir;
int  GRID_ROW_NUM;     // 15
int  GRID_COL_NUM;     // 20
int  SMOOTHNESS_ALPHA; // 20


#define DISTANCE 1
//#define STARE_MODE
//#define INTERMEDIATE_IMG
//#define DEBUG

struct Point {
	float x, y;
	Point() {}
	Point(float _x, float _y): x(_x), y(_y) {}
};

struct Matching {
    // from 1 to 2
    Point p1, p2;
    Matching(float _x1, float _y1, float _x2, float _y2) {
		Point _p1(_x1, _y1);
		p1 = _p1;
		Point _p2(_x2, _y2);
		p2 = _p2;
	}
};

struct Grid {
	Point p;
	float salience;
	Grid() {}
	Grid(Point _p): p(_p) { salience = 0.f; }
};


void read_matching(vector<Matching> *matching, char *name) {
	
    float x1, y1, x2, y2;
    
	FILE *f = fopen(name, "r");
	if (f == NULL) {
		printf("\n File open error:%s\n", name);
		exit(1);
	}
	int n = 0;
	fscanf(f, "%d", &n);
	for (int i = 0; i < n; ++i) {	
    //while(fscanf(f, "%f %f %f %f", &x1, &y1, &x2, &y2) != -1) {
		//fscanf(f, "%f %f %f %f", &x1, &y1, &x2, &y2);
        matching->push_back(Matching(x1, y1, x2, y2));        
        n++;
    }
	fclose(f);
	//printf("   Total # of features: %d\n", n);
}


void drawGrid(IplImage *dstImage, const vector< vector<Grid> > &grid) {
	for(unsigned int j = 0; j < grid.size(); j++) {
		for(unsigned int k = 0; k < grid[j].size(); k++) {
			if(k + 1 < grid[j].size())
				cvLine(dstImage, cvPoint((int)grid[j][k].p.x, (int)grid[j][k].p.y), cvPoint((int)grid[j][k + 1].p.x, (int)grid[j][k + 1].p.y), CV_RGB(255, 255, 255), 1, CV_AA, 0);
			if(j + 1 < grid.size())
				cvLine(dstImage, cvPoint((int)grid[j][k].p.x, (int)grid[j][k].p.y), cvPoint((int)grid[j + 1][k].p.x, (int)grid[j + 1][k].p.y), CV_RGB(255, 255, 255), 1, CV_AA, 0);
		}
	}
}

void warpPoint(Point *p, const float homography[9]) {
	float x = homography[0] * p->x + homography[1] * p->y + homography[2] * 1.f;
	float y = homography[3] * p->x + homography[4] * p->y + homography[5] * 1.f;
	float w = homography[6] * p->x + homography[7] * p->y + homography[8] * 1.f;
	p->x = x / w;
	p->y = y / w;
}

void preWarping(IplImage *dstImage, vector<Matching> *matching, vector< vector<Grid> > *grid, 
				const IplImage *srcImage, int imgIdx, bool isRightEyeView) {
	// init
	float *srcPointsData, *dstPointsData;
    srcPointsData = new float [matching->size() * 2];
    dstPointsData = new float [matching->size() * 2];
    for(unsigned int i = 0; i < matching->size(); i++) {
        srcPointsData[i * 2    ] = (*matching)[i].p1.x;
        srcPointsData[i * 2 + 1] = (*matching)[i].p1.y;
        dstPointsData[i * 2    ] = (*matching)[i].p2.x;
        dstPointsData[i * 2 + 1] = (*matching)[i].p2.y;
    }
    CvMat srcPoints = cvMat(matching->size(), 2, CV_32FC1, srcPointsData);
    CvMat dstPoints = cvMat(matching->size(), 2, CV_32FC1, dstPointsData);

	// calculate the homography matrix
	float homographyData[9];
    CvMat homography = cvMat(3, 3, CV_32FC1, homographyData);
    cvFindHomography(&srcPoints, &dstPoints, &homography, CV_RANSAC, 1, NULL);
	
	// apply the homography to the srcImage => initial guess warping!
	dstImage = cvCreateImage(cvGetSize(srcImage), IPL_DEPTH_8U, 3);
	cvWarpPerspective(srcImage, dstImage, &homography, CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS, cvScalarAll(0));
	
	// warp grid vertices and feature points with calculated homography
	for(unsigned int i = 0; i < matching->size(); i++) 
		warpPoint(&(*matching)[i].p1, homographyData);
	for(unsigned int i = 0; i < grid->size(); i++) 
		for(unsigned int j = 0 ; j < (*grid)[i].size(); j++) 
			warpPoint(&(*grid)[i][j].p, homographyData);
	
	cvReleaseImage(&dstImage);
	
	// free allocated memory
	delete [] srcPointsData;
    delete [] dstPointsData;
}

void findBoundingGrid(int *x, int *y, const Point &p, const vector< vector<Grid> > &grid) {
	// find the grid that contains the given feature point by calculating areas
	float min = 0.001f;
	for(unsigned int i = 0; i < grid.size() - 1; i++) {
		for(unsigned int j = 0; j < grid[i].size() - 1; j++) {
			Point v1 = grid[i][j].p, v2 = grid[i][j + 1].p, v3 = grid[i + 1][j + 1].p, v4 = grid[i + 1][j].p;
			// area of a quad: (v1,v2,v3,v4)
			float rect = abs( (v1.x * v2.y + v2.x * v3.y + v3.x * v4.y + v4.x * v1.y) 
						    - (v2.x * v1.y + v3.x * v2.y + v4.x * v3.y + v1.x * v4.y) );
			// area of triangles: (p,v1,v2), (p,v2,v3), (p,v3,v4), (p,v1,v4)
			float tri = abs( (p.x * v1.y + v1.x * v2.y + v2.x * p.y) 
						   - (v1.x * p.y + v2.x * v1.y + p.x * v2.y) );
			tri      += abs( (p.x * v2.y + v2.x * v3.y + v3.x * p.y) 
						   - (v2.x * p.y + v3.x * v2.y + p.x * v3.y) );
			tri      += abs( (p.x * v3.y + v3.x * v4.y + v4.x * p.y) 
						   - (v3.x * p.y + v4.x * v3.y + p.x * v4.y) );
			tri      += abs( (p.x * v1.y + v1.x * v4.y + v4.x * p.y) 
						   - (v1.x * p.y + v4.x * v1.y + p.x * v4.y) );
			// if the error is small enough => record x,y
			if(abs(tri - rect) < min) {
				min = abs(tri - rect);
				*x = j;
				*y = i;
			}
		}
	}
}

void getBilinearInterpolationWeight(float w[4], const Point &p, const vector< vector<Grid> > &grid, int x, int y) {
	Point v1 = grid[y][x].p, v2 = grid[y][x + 1].p, v3 = grid[y + 1][x + 1].p, v4 = grid[y + 1][x].p;
	// 0 for (x, y), 1 for (x + 1, y), 2 for (x + 1, y + 1), 3 for (x, y + 1)
	// Assumption: a initial-guess grid will normally be slightly defromed from a square grid by applying homography
	float L12 = sqrt((v1.x - v2.x) * (v1.x - v2.x) + (v1.y - v2.y) * (v1.y - v2.y));
	float L14 = sqrt((v1.x - v4.x) * (v1.x - v4.x) + (v1.y - v4.y) * (v1.y - v4.y));
	float a = (p.x - v1.x) / L12;
	float b = (p.y - v1.y) / L14;
	w[0] = (1.f - a) * (1.f - b);
	w[1] =        a  * (1.f - b);
	w[2] =	      a  *        b ;
	w[3] = (1.f - a) *        b ;
}

void contentPreservingWarping(IplImage *dstImage, vector< vector<Grid> > *dstGrid, 
							  const IplImage *srcImage, const vector< vector<Grid> > &srcGrid, const vector<Matching> &matching,
							  int imgIdx, bool isRightEyeView) {
	// init
	int imgWidth = srcImage->width;
	int imgHeight = srcImage->height;
	float colRatio = (float)imgWidth / GRID_COL_NUM;
	float rowRatio = (float)imgHeight / GRID_ROW_NUM;
	int nFeatures = matching.size();
	int nGrids = (GRID_ROW_NUM + 1) * (GRID_COL_NUM + 1);
	int nGrids2 = 2 * nGrids;
	// matrices
	float *xData = new float [nGrids2];
	float *bData = new float [2 * (nFeatures + nGrids)];
	float *aData = new float [(2 * (nFeatures + nGrids)) * nGrids2];
	CvMat x = cvMat(nGrids2, 1, CV_32FC1, xData);
	CvMat b = cvMat(2 * (nFeatures + nGrids), 1, CV_32FC1, bData);
	CvMat a = cvMat(2 * (nFeatures + nGrids), nGrids2, CV_32FC1, aData);
	cvSetZero(&x);
	cvSetZero(&b);
	cvSetZero(&a);
	// data term
	for(int k = 0; k < nFeatures; k++) {
		// b
		bData[2 * k    ] = matching[k].p2.x;
		bData[2 * k + 1] = matching[k].p2.y;
		// a
		// find out which grid does the feature falls in
		int x, y;
		findBoundingGrid(&x, &y, matching[k].p1, srcGrid);
		// compute bilnear interpolation weight
		float w[4];  // 0 for (x, y), 1 for (x + 1, y), 2 for (x + 1, y + 1), 3 for (x, y + 1)
		getBilinearInterpolationWeight(w, matching[k].p1, srcGrid, x, y);
		// Px 
		aData[((2 * k    ) * nGrids2) + (2 * ( y      * (GRID_COL_NUM + 1) +  x     ))    ] = w[0];
		aData[((2 * k    ) * nGrids2) + (2 * ( y      * (GRID_COL_NUM + 1) + (x + 1)))    ] = w[1];
		aData[((2 * k    ) * nGrids2) + (2 * ((y + 1) * (GRID_COL_NUM + 1) + (x + 1)))    ] = w[2];
		aData[((2 * k    ) * nGrids2) + (2 * ((y + 1) * (GRID_COL_NUM + 1) +  x     ))    ] = w[3];
		// Py
		aData[((2 * k + 1) * nGrids2) + (2 * ( y      * (GRID_COL_NUM + 1) +  x     )) + 1] = w[0];
		aData[((2 * k + 1) * nGrids2) + (2 * ( y      * (GRID_COL_NUM + 1) + (x + 1))) + 1] = w[1];
		aData[((2 * k + 1) * nGrids2) + (2 * ((y + 1) * (GRID_COL_NUM + 1) + (x + 1))) + 1] = w[2];
		aData[((2 * k + 1) * nGrids2) + (2 * ((y + 1) * (GRID_COL_NUM + 1) +  x     )) + 1] = w[3];
	}
	// similar transfrom term
	for(int j = 0; j < GRID_ROW_NUM + 1; j++) {
		for(int k = 0; k < GRID_COL_NUM + 1; k++) {
			float p, q, w;
			Point v1 = srcGrid[j][k].p, v2, v3;
			int v1Idx = j * (GRID_COL_NUM + 1) + k, v2Idx, v3Idx;
			// sum all neighbors
			for(int u = -1; u < 2; u += 2) {
				for(int v = -1; v < 2; v += 2) { 
					if((j + u >= 0) && (k + v >= 0) && (j + u < GRID_ROW_NUM + 1) && (k + v < GRID_COL_NUM + 1)) {
						if((u == -1) && (v == -1))
							w = srcGrid[j - 1][k - 1].salience;
						else if((u != -1) && (v == -1))
							w = srcGrid[j    ][k - 1].salience;
						else if((u == -1) && (v != -1))
							w = srcGrid[j - 1][k    ].salience;
						else 
							w = srcGrid[j    ][k    ].salience;	
						w *= SMOOTHNESS_ALPHA;

						v2 = srcGrid[j + u][k + v].p;
						v2Idx = (j + u) * (GRID_COL_NUM + 1) + (k + v);
						
						v3 = srcGrid[j + u][k].p;
						v3Idx = (j + u) * (GRID_COL_NUM + 1) +  k;
						p = ((v1.x - v2.x) * (v3.x - v2.x) + (v1.y - v2.y) * (v3.y - v2.y)) / ((v3.x - v2.x) * (v3.x - v2.x) + (v3.y - v2.y) * (v3.y - v2.y));
						q = ((v1.x - v2.x) * (v3.y - v2.y) - (v1.y - v2.y) * (v3.x - v2.x)) / ((v3.x - v2.x) * (v3.x - v2.x) + (v3.y - v2.y) * (v3.y - v2.y));
						// V1 x
						aData[(2 * (nFeatures + v1Idx) * nGrids2) + (2 * v1Idx)    ] += 1.f * w;
						aData[(2 * (nFeatures + v1Idx) * nGrids2) + (2 * v2Idx)    ] -= (1.f - p) * w;
						aData[(2 * (nFeatures + v1Idx) * nGrids2) + (2 * v3Idx)    ] -= p * w;
						aData[(2 * (nFeatures + v1Idx) * nGrids2) + (2 * v2Idx) + 1] += q * w;
						aData[(2 * (nFeatures + v1Idx) * nGrids2) + (2 * v3Idx) + 1] -= q * w;
						// V1 y
						aData[((2 * (nFeatures + v1Idx) + 1) * nGrids2) + (2 * v1Idx) + 1] += 1.f * w;
						aData[((2 * (nFeatures + v1Idx) + 1) * nGrids2) + (2 * v2Idx) + 1] -= (1.f - p) * w;
						aData[((2 * (nFeatures + v1Idx) + 1) * nGrids2) + (2 * v3Idx) + 1] -= p * w;
						aData[((2 * (nFeatures + v1Idx) + 1) * nGrids2) + (2 * v2Idx)    ] -= q * w;
						aData[((2 * (nFeatures + v1Idx) + 1) * nGrids2) + (2 * v3Idx)    ] += q * w;
				
						v3 = srcGrid[j][k + v].p;
						v3Idx =  j * (GRID_COL_NUM + 1) + (k + v);
						p = ((v1.x - v2.x) * (v3.x - v2.x) + (v1.y - v2.y) * (v3.y - v2.y)) / ((v3.x - v2.x) * (v3.x - v2.x) + (v3.y - v2.y) * (v3.y - v2.y));
						q = ((v1.x - v2.x) * (v3.y - v2.y) - (v1.y - v2.y) * (v3.x - v2.x)) / ((v3.x - v2.x) * (v3.x - v2.x) + (v3.y - v2.y) * (v3.y - v2.y));
						// V1 x
						aData[(2 * (nFeatures + v1Idx) * nGrids2) + (2 * v1Idx)    ] += 1.f * w;
						aData[(2 * (nFeatures + v1Idx) * nGrids2) + (2 * v2Idx)    ] -= (1.f - p) * w;
						aData[(2 * (nFeatures + v1Idx) * nGrids2) + (2 * v3Idx)    ] -= p * w;
						aData[(2 * (nFeatures + v1Idx) * nGrids2) + (2 * v2Idx) + 1] += q * w;
						aData[(2 * (nFeatures + v1Idx) * nGrids2) + (2 * v3Idx) + 1] -= q * w;
						// V1 y
						aData[((2 * (nFeatures + v1Idx) + 1) * nGrids2) + (2 * v1Idx) + 1] += 1.f * w;
						aData[((2 * (nFeatures + v1Idx) + 1) * nGrids2) + (2 * v2Idx) + 1] -= (1.f - p) * w;
						aData[((2 * (nFeatures + v1Idx) + 1) * nGrids2) + (2 * v3Idx) + 1] -= p * w;
						aData[((2 * (nFeatures + v1Idx) + 1) * nGrids2) + (2 * v2Idx)    ] -= q * w;
						aData[((2 * (nFeatures + v1Idx) + 1) * nGrids2) + (2 * v3Idx)    ] += q * w;
					}
				}
			}
		}
	}
	// solve linear system for dstGrid
	cvSolve(&a, &b, &x, CV_SVD);	

	// save result grid coordinates 
	for(int i = 0; i < GRID_ROW_NUM + 1; i++) {
		vector<Grid> tmp;
		for(int j = 0; j < GRID_COL_NUM + 1; j++)  
			tmp.push_back(Point(xData[2 * (i * (GRID_COL_NUM + 1) + j)], xData[2 * (i * (GRID_COL_NUM + 1) + j) + 1]));
		dstGrid->push_back(tmp);
	}	
	// show result on the window
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode (GL_MODELVIEW);
	glLoadIdentity();

	int IMG_WIDTH  = srcImage->width;
	int IMG_HEIGHT = srcImage->height;
	// Access openGL texture data
	for (int i=0; i < GRID_ROW_NUM; ++i) {
		for (int j=0; j < GRID_COL_NUM; ++j) {
			// flip y direction since opengl has (0,0) on the lower left rather than top left
			float texture_upper_left_corner_x = (float)(j * colRatio / IMG_WIDTH);
			float texutre_upper_left_corner_y = (float)(i * rowRatio / IMG_HEIGHT);
			float texture_lower_left_corner_x = (float)((j+1) * colRatio / IMG_WIDTH);
			float texutre_lower_left_corner_y = (float)((i+1) * rowRatio / IMG_HEIGHT);

			glBegin(GL_QUADS);						
			glTexCoord2f(texture_upper_left_corner_x, texutre_lower_left_corner_y); // lower left corner of image 
			glVertex2f((*dstGrid)[i + 1][j].p.x - (imgWidth / 2), -((*dstGrid)[i + 1][j].p.y - (imgHeight / 2)));
			glTexCoord2f(texture_lower_left_corner_x, texutre_lower_left_corner_y); // lower right corner of image 
			glVertex2f((*dstGrid)[i + 1][j + 1].p.x - (imgWidth / 2), -((*dstGrid)[i + 1][j + 1].p.y - (imgHeight / 2)));
			glTexCoord2f(texture_lower_left_corner_x, texutre_upper_left_corner_y); // upper right corner of image 
			glVertex2f((*dstGrid)[i][j + 1].p.x - (imgWidth / 2), -((*dstGrid)[i][j + 1].p.y - (imgHeight / 2)));
			glTexCoord2f(texture_upper_left_corner_x, texutre_upper_left_corner_y); // upper left corner of image 
			glVertex2f((*dstGrid)[i][j].p.x - (imgWidth / 2), -((*dstGrid)[i][j].p.y - (imgHeight / 2)));
			glEnd();
		}
	}

	// draw output images
	//char name[255];
	uchar *dstImageData = new uchar [imgWidth * imgHeight * 3];
	//IplImage *dstImage_tmp = cvCreateImage(cvGetSize(srcImage), IPL_DEPTH_8U, 3);
	cvSetZero(dstImage);
	// get image from memory
	glReadPixels(0, 0, imgWidth, imgHeight, GL_RGB, GL_UNSIGNED_BYTE, dstImageData);
	for(int i = 0; i < imgHeight; i++)		
		memcpy(&dstImage->imageData[i * imgWidth * 3], &dstImageData[(imgHeight - i - 1) * imgWidth * 3], imgWidth * 3);
	cvCvtColor(dstImage, dstImage, CV_RGB2BGR);


	glFlush(); 
	// free allocated memory
	delete [] xData;
	delete [] bData;
	delete [] aData;
	delete [] dstImageData;	
}


void reshape(int w, int h) {
    glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
	gluOrtho2D(-w * 0.5f, w * 0.5f, -h * 0.5f, h * 0.5f);
}


void display() {
	
	printf("Content preserving warps: 0.00");
	for(int i = 0; i < num_frame; i++) {	
		char name[255];		
		printf("\b\b\b\b%.2f", (float)((i+1)/num_frame));		
		
		// initialization and declaration
		vector<Matching> matching;		
		sprintf(name, "%s%04d.matching", input_matching_dir, i);
		read_matching(&matching, name);


		// load original image		
		sprintf(name, "%s%04d.jpg", input_image_dir, i);
		IplImage *srcImage = cvLoadImage(name);

		// initialize grids
		vector< vector<Grid> > srcGrid;
		vector< vector<Grid> > dstGrid;

		float colRatio = (float)srcImage->width / GRID_COL_NUM;
		float rowRatio = (float)srcImage->height / GRID_ROW_NUM;
		// coordinates of grids
		for(int j = 0; j < GRID_ROW_NUM + 1; j++) {
			
			vector<Grid> tmp;
			for(int k = 0; k < GRID_COL_NUM + 1; k++) { 
				tmp.push_back(Point(k * colRatio, j * rowRatio));				
			}
			srcGrid.push_back(tmp);
		}
		// salience of grids
		for(int j = 0; j < GRID_ROW_NUM; j++) {
			for(int k = 0; k < GRID_COL_NUM; k++) {
				float mean = 0.f, var = 0.f;
				// calculate grid mean
				for(int u = j * rowRatio; u < (j + 1) * rowRatio; u++) {
					for(int v = k * colRatio; v < (k + 1) * colRatio; v++) {
						mean += (0.11f * (((uchar *)(srcImage->imageData + u*srcImage->widthStep))[v*srcImage->nChannels + 0])
							   + 0.59f * (((uchar *)(srcImage->imageData + u*srcImage->widthStep))[v*srcImage->nChannels + 1])
							   + 0.3f  * (((uchar *)(srcImage->imageData + u*srcImage->widthStep))[v*srcImage->nChannels + 2]));
					}
				}
				mean /= (rowRatio * colRatio);
				// calculate grid variance
				for(int u = j * rowRatio; u < (j + 1) * rowRatio; u++) {
					for(int v = k * colRatio; v < (k + 1) * colRatio; v++) {
						float gray = (0.11f * (((uchar *)(srcImage->imageData + u*srcImage->widthStep))[v*srcImage->nChannels + 0])
							        + 0.59f * (((uchar *)(srcImage->imageData + u*srcImage->widthStep))[v*srcImage->nChannels + 1])
							        + 0.3f  * (((uchar *)(srcImage->imageData + u*srcImage->widthStep))[v*srcImage->nChannels + 2]));
						var += ((gray - mean) * (gray - mean));
					}
				}
				var /= (rowRatio * colRatio);

				// grid salience = l2-norm of variance + epsilon
				srcGrid[j][k].salience = sqrt(var) + 0.5f;				
			}
		}	
				
		CvSize size = cvSize(TEXTURE_WIDTH, TEXTURE_HEIGHT); 		
		IplImage *texture_data = cvCreateImage(size, srcImage->depth, srcImage->nChannels);
		cvResize(srcImage, texture_data, CV_INTER_LINEAR);
		cvCvtColor(texture_data, texture_data, CV_BGR2RGB);

		
		// load grid textures with OpenGL
		GLuint texture_id;
		glGenTextures(1, &texture_id);

		glBindTexture(GL_TEXTURE_2D, texture_id);
		glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, TEXTURE_WIDTH, TEXTURE_HEIGHT, 0, GL_RGB, GL_UNSIGNED_BYTE, texture_data->imageDataOrigin);
		

		// draw features on original image
		IplImage *dstImage = cvCloneImage(srcImage);

		// pre-warping
		preWarping(dstImage, &matching, &srcGrid, srcImage, i, true);
		
		// content-preserving warping
		contentPreservingWarping(dstImage, &dstGrid, srcImage, srcGrid, matching, i, true);
		
		// write output image
		char outname[255];
		sprintf(outname, "%s%04d.jpg", output_image_dir, i);
		cvSaveImage(outname, dstImage);

		// free
		cvReleaseImage(&srcImage);
		cvReleaseImage(&dstImage);		
		//printf("*** Image %s%04d.jpg is warped.\n", output_image_dir, i);
	}
	printf("\n");
	// Break GLUT loop for bash running this program.
	exit(1);
}

void keyboardFunc(unsigned char key, int x, int y) {
    switch (key) {
        /* exit the program */
        case 27:
        case 'q':
		case 'Q': 
            exit(0);
        break;
    }
}

void init() {
	glClearColor(0.f, 0.f, 0.f, 0.f);
	glEnable(GL_TEXTURE_2D); 
}


void help(char* function_name)
{
	printf("Usage: %s <input_image_dir> <input_matching_dir> <output_image_dir> <num_frame> \n", function_name);
	exit(1);
}

void defalut_setting(void)
{
	printf("Using default setting:\n \
		   GRID_ROW_NUM = 15\n\
		   GRID_COL_NUM = 20\n\
		   SMOOTHNESS_ALPHA = 20\n");
	GRID_ROW_NUM     = 15;  
	GRID_COL_NUM     = 20;  
	SMOOTHNESS_ALPHA = 20;
}

//#define DEBUG

int main(int argc, char **argv) {

#ifndef DEBUG
	if (argc < 5) help(argv[0]);
	else if (argc < 8) defalut_setting();
	
	input_image_dir   = argv[1];
	input_matching_dir= argv[2];
    output_image_dir  = argv[3];
	num_frame         = atoi(argv[4]);
#else
	num_frame         = 3;
	input_image_dir   = "../dat/csiegirl_l/";
	input_matching_dir= "../log/csiegirl_l/";
    output_image_dir  = "../dat/csiegirl_l_cpw/";
	defalut_setting();
#endif
		
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(640, 480);
	glutInitWindowPosition(300, 300);
	glutCreateWindow("Content-Preserving Warping");
	init();
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	//glutKeyboardFunc(keyboardFunc);
	glutMainLoop(); 
    return 0;
}  
