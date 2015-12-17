// Matching two .key files from David Lowe's sift binary
#include <cstdio>
#include <iostream>
#include <fstream>
using std::cerr;
using std::endl;
using std::ofstream;

#include <cv.h>
#include <highgui.h>

#include "sift_feature.h"
using SIFT::FeaturePoint;
using SIFT::FeatureList;
using SIFT::FeatureListLoader;

#include "ANN.h"
#include <cmath>
#include <vector>
using std::vector;

#define NO_OVERLAP

static double FUND_ERROR_THRESHOLD = 1e-3;

static void buildTree( ANNpointArray &dataPts, ANNkd_tree *&tree, const FeatureList &fl ) {

    const unsigned int nPts = fl.size();
    const unsigned int dim = FeaturePoint::NUM_OF_DIMENSION;
    dataPts = annAllocPts( nPts, dim );
    for (unsigned int i = 0; i < nPts; ++i) {
	for (unsigned int j = 0; j < dim; ++j) {
	    dataPts[i][j] = fl[i].desc[j];
	}
    }

    tree = new ANNkd_tree( dataPts, nPts, dim );
}

static unsigned int _siftMatch( const FeatureList &feature_list1, const FeatureList &feature_list2,
	FeatureList &matched_list1, FeatureList &matched_list2) { 
    ANNpointArray dataPts1;
    ANNpointArray dataPts2;

    ANNkd_tree *tree1;
    ANNkd_tree *tree2;

    buildTree( dataPts1, tree1, feature_list1 );
    buildTree( dataPts2, tree2, feature_list2 );

    unsigned int matched = 0;
    const unsigned int dim = FeaturePoint::NUM_OF_DIMENSION;
    for (unsigned int i = 0; i < feature_list1.size(); ++i) {
	ANNdist *dists  = new ANNdist[2];
	ANNidx *nn_idx1 = new ANNidx[2];
	ANNidx *nn_idx2 = new ANNidx[1];

	// query point, #near neighbors, neighbors (ret), distance (ret), error bound
	tree2->annkSearch( dataPts1[i], 2, nn_idx2, dists, 0 ); 

	if (10 * 10 * dists[0] < 6 * 6 * dists[1]) { // sift criteria

	    tree1->annkSearch( dataPts2[nn_idx2[0]], 1, nn_idx1, dists, 0 );
	    if ( nn_idx1[0] == i ) { // double check

		//if ( abs(feature_list1.at(i).y - feature_list2.at(nn_idx2[0]).y) <= 1.0 ) { // horizontal matching constraint 
		    matched_list1.push_back( feature_list1.at(i) );
		    matched_list2.push_back( feature_list2.at(nn_idx2[0]) );
		    ++matched;
		//}
	    }
	}

	delete [] dists;
	delete [] nn_idx1;
	delete [] nn_idx2;

    }

    return matched;
}
/*
void FitFundamentalMatrix( const FeatureList &fl1, const FeatureList &fl2,
		           CvMat *R ) {
    // Fitting a 3x3 fundamental matrix by 8-points algorithm

    // Fill A
    const int num_of_observation = 8;
    CvMat* A = cvCreateMat( num_of_observation, 9, CV_32FC1 );
    for (int i = 0; i < num_of_observation; ++i) {
	p1 = fl1.at(i);
	p2 = fl2.at(i);

	cvmSet(i, 0, p2.x * p1.x );
	cvmSet(i, 1, p2.x * p1.y );
	cvmSet(i, 2, p2.x );
	cvmSet(i, 3, p2.y * p1.x );
	cvmSet(i, 4, p2.y * p1.y );
	cvmSet(i, 5, p2.y );
	cvmSet(i, 6, p1.x );
	cvmSet(i, 7, p1.y );
	cvmSet(i, 8, 1.0 );
    }
   
    // SVD A
    cvMat* U = cvCreateMat( num_of_observation, num_of_observation, CV_32FC1 );
    cvMat* D = cvCreateMat( num_of_observation, 9, CV_32FC1 );
    cvMat* V = cvCreateMat( 9, 9, CV_32FC1 );
    cvSVD( A, D, U, V, CV_SVD_U_T|CV_SVD_V_T ); // A = U D V^T

    // Extract fundamental matrix from the column of V
    // corresponding to the smallest singular value.
    double a[9];
    for (int i = 0; i < 9; ++i )
	a[i] = cvmGet( V, i, 8 );
	
    cvMat F = cvMat( 3, 3, CV_32FC1, a );
    cvReleaseMat( &U );
    cvReleaseMat( &D );
    cvReleaseMat( &V );

    // Enforce rank2 constraint
    cvMat* U = cvCreateMat( 3, 3, CV_32FC1 );
    cvMat* D = cvCreateMat( 3, 3, CV_32FC1 );
    cvMat* V = cvCreateMat( 3, 3, CV_32FC1 );
    cvSVD( F, D, U, V, CV_SVD_U_T|CV_SVD_V_T );

    cvmSet( D, 3, 3, 0.0 ); // rank2
    cvMat* T = cvCreateMat( 3, 3, CV_32FC1 );
    cvMat* VT = cvCreateMat( 3, 3, CV_32FC1 );
    cvTranspose( V, VT );
    cvMatMul( D, VT, T );
    cvMatMul( U, T, R );

    cvReleaseMat( &U );
    cvReleaseMat( &D );
    cvReleaseMat( &V );
    cvReleaseMat( &F );
    cvReleaseMat( &A );
    
}
*/
int findFundamentalMat( const FeatureList &fl1, const FeatureList &fl2,
	                 CvMat* fundMatr,
			 FeatureList &fl_in1, FeatureList &fl_in2) 
{
    int num_points = fl1.size();

    CvMat* points1;
    CvMat* points2;
    CvMat* status;
    
    points1 = cvCreateMat( 2, num_points, CV_32F );
    points2 = cvCreateMat( 2, num_points, CV_32F );
    status  = cvCreateMat( 1, num_points, CV_8UC1 );

    for (int i = 0; i < num_points; ++i) {
	FeaturePoint p1 = fl1.at(i);
	FeaturePoint p2 = fl2.at(i);

	cvmSet( points1, 0, i, p1.x );
	cvmSet( points1, 1, i, p1.y );
	cvmSet( points2, 0, i, p2.x );
	cvmSet( points2, 1, i, p2.y );
    }


    int in_num = 0;
    int fm_count = cvFindFundamentalMat( points1, points2, fundMatr, CV_FM_RANSAC, 1.0, 0.99, status );
    if (fm_count == 1) {

	for( int i = 0; i < num_points; ++i ) {
	    FeaturePoint p1 = fl1.at(i);
	    FeaturePoint p2 = fl2.at(i);

	    CvMat * tmp_point1 = cvCreateMat(3, 1, CV_32F);
	    CvMat * tmp_point2 = cvCreateMat(3, 1, CV_32F);
	    cvmSet( tmp_point1, 0, 0, p1.x );
	    cvmSet( tmp_point1, 1, 0, p1.y );
	    cvmSet( tmp_point1, 2, 0, 1.0 );
	    cvmSet( tmp_point2, 0, 0, p2.x );
	    cvmSet( tmp_point2, 1, 0, p2.y );
	    cvmSet( tmp_point2, 2, 0, 1.0 );

	    CvMat * tmp_result = cvCreateMat(3, 1, CV_32F);
	    cvMatMul( fundMatr, tmp_point1, tmp_result );
	    CvMat * tmp_point2T = cvCreateMat(1, 3, CV_32F);
	    cvTranspose( tmp_point2, tmp_point2T );
	    CvMat * tmp_error = cvCreateMat(1, 1, CV_32F);
	    cvMatMul( tmp_point2T, tmp_result, tmp_error );

	    double error = cvmGet( tmp_error, 0, 0 );
	    if (error < FUND_ERROR_THRESHOLD) {
		fl_in1.push_back(p1);
		fl_in2.push_back(p2);
		++in_num;
	    }
	    
	}

    } else {
	fl_in1 = fl1;
	fl_in2 = fl2;
    }
    return in_num;
}

void writeKeyFile( const char *output_file,
		   //const char *output_key_dir, const char *image_name, const int frame_num, const char image_postfix,
	           const FeatureList &feature_list ) {

    ofstream my_file;
    my_file.open( output_file );
    const unsigned int num_matched = feature_list.size();
    my_file << num_matched << " " << FeaturePoint::NUM_OF_DIMENSION << std::endl;
    for (int i = 0; i < num_matched; ++i) {
	FeaturePoint fp = feature_list.at(i);
	my_file << fp.y << " " << fp.x << " " << fp.scale << " " << fp.orientation << std::endl;
	for (int j = 0; j < 6; ++j) {
	    for (int k = 0; k < 20; ++k) {
		my_file << " " << (unsigned int)fp.desc[j*20+k];
	    }
	    my_file << std::endl;
	}
	for (int j = 0; j < 8; ++j) {
	    my_file << " "<< (unsigned int)fp.desc[120+j];
	}
	my_file << std::endl;	    
    }

    my_file.close();
}

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

int main( int argc, char *argv[] ) 
{
    if (argc != 6) {
	std::cerr << "Usage: " << argv[0]
	    << " <input_key_dir1> <input_key_dir2> <output_key_dir1> <output_key_dir2> <output_fund_mat_dir>"
	    << std::endl;
	return -1;
    }

    const char *input_key_dir1	= argv[1];
    const char *input_key_dir2	= argv[2];
    const char *output_key_dir1 = argv[3];
    const char *output_key_dir2 = argv[4];
    const char *output_fund_mat_dir = argv[5];

    char image_postfix;
    char image_name[128];
    parseFileName( argv[1], image_name, image_postfix );

    FeatureListLoader feature_list_loader1( input_key_dir1, image_name, 'l' );
    FeatureListLoader feature_list_loader2( input_key_dir2, image_name, 'r' );

    int frame_num = -1;
    while (1) {

	++ frame_num;

	bool file1_is_not_exist, file2_is_not_exist;	

	FeatureList feature_list1;
	file1_is_not_exist = feature_list_loader1.load( feature_list1, frame_num );

	FeatureList feature_list2;
	file2_is_not_exist = feature_list_loader2.load( feature_list2, frame_num );

	if (file1_is_not_exist || file2_is_not_exist) break;
	
	char output_file1[128], output_file2[128], output_fileF[128];
	sprintf( output_file1, "%s%s_%04d_l.key", output_key_dir1, image_name, frame_num );
	sprintf( output_file2, "%s%s_%04d_r.key", output_key_dir2, image_name, frame_num );
	sprintf( output_fileF, "%s%s_%04d.fund", output_fund_mat_dir, image_name, frame_num );
#ifdef NO_OVERLAP
    
	if ( access( output_file1, F_OK )==0 && 
	     access( output_file2, F_OK )==0 &&
	     access( output_fileF, F_OK )==0 ) continue;
#endif
	// if (frame_num == 1) break; // debug 

	// Matching feature_list1 and feature_list2
	unsigned int num_matched = 0;
	
	FeatureList matched_list1;
	FeatureList matched_list2;
	num_matched = _siftMatch( feature_list1, feature_list2, matched_list1, matched_list2 );
	//cerr << "matched before ransac " << num_matched << endl;


	FeatureList in_list1;
	FeatureList in_list2;
	CvMat * fundMatr = cvCreateMat(3, 3, CV_32F);
	num_matched = findFundamentalMat( matched_list1, matched_list2, fundMatr, in_list1, in_list2 );
	
	writeKeyFile( output_file1, in_list1 );
	writeKeyFile( output_file2, in_list2 );
	    
	ofstream my_file;
	my_file.open( output_fileF );

	my_file << cvmGet(fundMatr, 0, 0) << " "
		<< cvmGet(fundMatr, 0, 1) << " "
		<< cvmGet(fundMatr, 0, 2) << " "
		<< cvmGet(fundMatr, 1, 0) << " "
		<< cvmGet(fundMatr, 1, 1) << " "
		<< cvmGet(fundMatr, 1, 2) << " "
		<< cvmGet(fundMatr, 2, 0) << " "
		<< cvmGet(fundMatr, 2, 1) << " "
		<< cvmGet(fundMatr, 2, 2) ;
	my_file.close();

	std::cerr << "Feature points matched: " << num_matched << std::endl;
	
    }

    /*
    char output_fund_mat_file[128];
    sprintf( output_fund_mat_file, "%s%s.fund", output_fund_mat_dir, image_name );
    ofstream my_file;
    my_file.open( output_fund_mat_file );
    my_file << fund_mat_list.size() << " " << 9 << std::endl;
    for (int i = 0; i < fund_mat_list.size(); ++i) {
	const CvMat * m = fund_mat_list.at(i);
	my_file << cvmGet(m, 0, 0) << " ";
	my_file << cvmGet(m, 0, 1) << " ";
	my_file << cvmGet(m, 0, 2) << " ";
	my_file << cvmGet(m, 1, 0) << " ";
	my_file << cvmGet(m, 1, 2) << " ";
	my_file << cvmGet(m, 1, 2) << " ";
	my_file << cvmGet(m, 2, 0) << " ";
	my_file << cvmGet(m, 2, 1) << " ";
	my_file << cvmGet(m, 2, 2) << std::endl;
    }
    my_file.close();
*/
    return 0;
}
