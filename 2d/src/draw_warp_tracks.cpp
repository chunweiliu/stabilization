#include <highgui.h> // should put in the front

#include <iostream>
#include <fstream>

#include "sift_feature.h"
#include "sift_particle_video.h"
#include "sift_stabilize.h"
#include "cv.h"


using std::vector;
using std::map;
using ParticleVideo::ParticleMaps;
using ParticleVideo::Particle;
using ParticleVideo::ParticleIdx;
using ParticleVideo::Trajactory;
using SIFT::FeaturePoint;
using SIFT::FeatureList;
using SIFT::FeatureListLoader;
typedef ParticleMaps<FeaturePoint>      SIFTParticleMaps;
typedef Particle<FeaturePoint>::Map     SIFTParticleMap;
typedef Trajactory<FeaturePoint>        ParticleTrajactory;

typedef Particle<FeaturePoint>	        SIFTParticle;

std::map<std::pair<int, ParticleIdx>, double> WeightMap;
static const unsigned int MIN_FILTERED = 5;
static const unsigned int FILTERED = 30;
static const double DELTA_REG = 1e-6;
static const double SCALE_TOR = 1e-1;
static const double SIGMA_SMOOTH = 10;
static const double LAMBDA_SMOOTH = 0;

unsigned int PLOT_TRACKS_LENGTH;
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

void filtered_tracks(
	vector< vector<ParticleIdx> > &filtered_particles,
	const SIFTParticleMaps &pms) {
    // {{{
    for (int frame_num = 1; frame_num < (signed)pms.size()-1; ++frame_num) {
	vector< std::pair<double, ParticleIdx> > list; 
	const SIFTParticleMap &pm = pms[frame_num];
	std::vector<ParticleIdx> particles;
	for (
		SIFTParticleMap::const_iterator p = pm.begin();
		p != pm.end();
		++p) {
	    ParticleIdx particle_num = p->first;

	    if (
		    pms.exist(frame_num-1, particle_num) &&
		    pms.exist(frame_num+1, particle_num)) {

		double cost = 0;
		double weight_sum = 0;
		{
		    const CvPoint2D32f p0 = pms.getPosition(frame_num-1, particle_num);
		    const CvPoint2D32f p1 = pms.getPosition(frame_num, particle_num);
		    const CvPoint2D32f p2 = pms.getPosition(frame_num+1, particle_num);
		    const double ax = p0.x-2*p1.x+p2.x;
		    const double ay = p0.y-2*p1.y+p2.y;
		    const double vx = p1.x-p2.x;
		    const double vy = p1.y-p2.y;
		    cost +=  (ax*ax+ay*ay);
		    weight_sum += 1;
		}

		int length = 2;
		while (
			pms.exist(frame_num+length, particle_num) &&
			pms.exist(frame_num-length, particle_num)) {
		    {
			const CvPoint2D32f p0 =
			    pms.getPosition(frame_num+length-2, particle_num);
			const CvPoint2D32f p1 =
			    pms.getPosition(frame_num+length-1, particle_num);
			const CvPoint2D32f p2 =
			    pms.getPosition(frame_num+length, particle_num);

			const double ax = p0.x-2*p1.x+p2.x;
			const double ay = p0.y-2*p1.y+p2.y;
			const double vx = p1.x-p2.x;
			const double vy = p1.y-p2.y;
			cost +=  exp(-(length*length))*(ax*ax+ay*ay);
			weight_sum += exp(-(length*length));
		    }

		    {
			const CvPoint2D32f p0 =
			    pms.getPosition(frame_num-length+2, particle_num);
			const CvPoint2D32f p1 =
			    pms.getPosition(frame_num-length+1, particle_num);
			const CvPoint2D32f p2 =
			    pms.getPosition(frame_num-length, particle_num);

			const double ax = p0.x-2*p1.x+p2.x;
			const double ay = p0.y-2*p1.y+p2.y;
			const double vx = p1.x-p2.x;
			const double vy = p1.y-p2.y;
			cost +=  exp(-(length*length))*(ax*ax+ay*ay);
			weight_sum += exp(-(length*length)/SIGMA_SMOOTH/SIGMA_SMOOTH);
		    }
		    ++length;
		}; 
		cost /= weight_sum;
		list.push_back(std::make_pair(length/(1+LAMBDA_SMOOTH*cost), particle_num));
	    }
	}

	for (unsigned int i = 0; i < list.size(); ++i) {
	    const ParticleIdx particle_num = list[i].second;
	    const CvPoint2D32f p1 = pms.getPosition(frame_num, particle_num);
	    double sum = 0;
	    for (unsigned int j = 0; j < list.size(); ++j) {
		const CvPoint2D32f p2 = pms.getPosition(frame_num, list[j].second);
		const double dx = p1.x-p2.x;
		const double dy = p1.y-p2.y;
		const double squared_dist = dx*dx+dy*dy;
		sum += exp(-squared_dist/900);
	    }
	    list[i].first /= sum;
	}

	{
	    sort(list.rbegin(), list.rend());
	    for (unsigned int i = 0; i < FILTERED && i < list.size(); ++i) {
		const ParticleIdx particle_num = list[i].second;
		particles.push_back(particle_num);
		WeightMap[std::make_pair(frame_num, particle_num)] = 
		    list[i].first;
	    }
	}

	{
	    double sum = 0;
	    for (unsigned int i = 0; i < particles.size(); ++i) {
		sum += WeightMap[std::make_pair(frame_num, particles[i])];
	    }

	    for (unsigned int i = 0; i < particles.size(); ++i) {
		WeightMap[std::make_pair(frame_num, particles[i])] /= sum;
	    }
	}
	filtered_particles[frame_num] = particles;

	if (filtered_particles[frame_num].size() < MIN_FILTERED) {
	    std::cerr << frame_num  << " has only " << filtered_particles[frame_num].size() << " tracks." <<
		std::endl;
	} 
    }
    // }}}
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

void plotTracks( const char* tracks_file, IplImage *output) {
    
    ifstream in(tracks_file); 
    while (true) {
	Trajactory<FeaturePoint> t(in);
	if (in.eof()) break;

	const vector<FeaturePoint> &plist = t.getList();
	if (t.length() > PLOT_TRACKS_LENGTH) {
	   for (unsigned int i = 0; i < t.length()-1; ++i) {

	       const FeaturePoint &p = plist.at(i);
	       const FeaturePoint &next_p = plist.at(i+1);
	       
	       cvLine( output,
		       cvPoint(p.x, p.y),
		       cvPoint(next_p.x, next_p.y),
		       CV_RGB(255, 0, 0), 1, CV_AA ) ;
	   }
	}
    }

}

void plotWarpTracks( vector<CvMat*> transformList, const char* tracks_file, 
		     IplImage *output1, IplImage *output2) {

    //srand ( time(NULL) );

    ifstream in( tracks_file );
    while (true) {
	Trajactory<FeaturePoint> t(in);
	if (in.eof()) break;

	const vector<FeaturePoint> &plist = t.getList();
	if (t.length() > PLOT_TRACKS_LENGTH) {
	    //const int color_r = rand()%256+1;
	    //const int color_g = rand()%256+1;
	    //const int color_b = rand()%256+1;
	    const int color_r = 0;
	    const int color_g = 0;
	    const int color_b = 255;

	   for (unsigned int i = 0; i < t.length()-1; ++i) {

	       const FeaturePoint &p = plist.at(i);
	       const FeaturePoint &next_p = plist.at(i+1);
	       
	       cvLine( output1,
		       cvPoint(p.x, p.y),
		       cvPoint(next_p.x, next_p.y),
		       CV_RGB(color_r, color_g, color_b), 1, CV_AA ) ;


	       CvMat* warp_x = cvCreateMat(2, 1, CV_32FC1);
	       CvMat* M = transformList.at( t.begin() + i );
	       double a, b, dx, dy;
	       a = cvmGet(M, 0, 0);
	       b = cvmGet(M, 1, 0);
	       dx = cvmGet(M, 0, 2);
	       dy = cvmGet(M, 1, 2);

	       CvMat* warp_next_x = cvCreateMat(2, 1, CV_32FC1);
	       CvMat* next_M = transformList.at( t.begin() + i+1 );
	       double next_a, next_b, next_dx, next_dy;
	       next_a = cvmGet(next_M, 0, 0);
	       next_b = cvmGet(next_M, 1, 0);
	       next_dx = cvmGet(next_M, 0, 2);
	       next_dy = cvmGet(next_M, 1, 2);

	       //a = 1.0; b = 0.0; dx = 0.0; dy = 0.0;

	       cvmSet( warp_x, 0, 0, a*p.x - b*p.y + dx );
	       cvmSet( warp_x, 1, 0, b*p.x + a*p.y + dy );
	       cvmSet( warp_next_x, 0, 0, next_a*next_p.x - next_b*next_p.y + next_dx );
	       cvmSet( warp_next_x, 1, 0, next_b*next_p.x + next_a*next_p.y + next_dy );

	       cvLine( output2,
		       cvPoint(cvmGet(warp_x, 0, 0), cvmGet(warp_x, 1, 0)),
		       cvPoint(cvmGet(warp_next_x, 0, 0), cvmGet(warp_next_x, 1, 0)),
		       CV_RGB(color_r, color_g, color_b), 1, CV_AA );
		       
	   }
	}
    }
}

int main( int argc, char *argv[] ) 
{
    if (argc != 7) {
	cerr << "Usage: " << argv[0]
	     << " <input_image_file> <tracks_file> <path_file> <output_image_file1> <output_image_file2> <plot_tracks_length>"
	     << endl;
	return -1;
    }
    const char* image_file = argv[1];
    const char* tracks_file = argv[2];
    const char* path_file = argv[3];
    const char* output_file1 = argv[4];
    const char* output_file2 = argv[5];
    const char* plot_tracks_length = argv[6];

    
    PLOT_TRACKS_LENGTH = atoi(plot_tracks_length);
    //const int MODE = atoi(mode);

    //char image_postfix;
    //char image_name[128];
    //parseFileName( argv[1], image_name, image_postfix );

    //char image_file[128];
    //sprintf( image_file, "%s%s_%04d_%c.png", image_dir, image_name, KEY_FRAME, image_postfix );
    IplImage *frame = cvLoadImage( image_file );
    if (frame == NULL) {
	std::cerr << image_file << " did not exist." << std::endl;
	return -1;
    }
    IplImage *output_frame1 = cvCloneImage( frame );
    IplImage *output_frame2 = cvCloneImage( frame );

    // read path file
    
    vector<CvMat*> transformList;
    loadTransformMatrices(transformList, path_file);

    // read and draw tracks file
    plotWarpTracks( transformList, tracks_file, output_frame1, output_frame2 );
    /*
    if (MODE == 0)
	plotTracks( tracks_file, output_frame );
    else
	plotWarpTracks( transformList, tracks_file, output_frame );
*/
    //SIFTParticleMaps pms; // Using SIFT as ParticleType
    //load_tracks(pms, tracks_file);
    //vector< vector<ParticleIdx> > filtered_particles(pms.size());
    //filtered_tracks(filtered_particles, pms);

    // draw tracks, and tracks after apply warp
 /* 
    for (unsigned int i = 0; i < pms.size(); ++i) {
	const SIFTParticleMap &pm = pms[i];
	// for each frame, do
	
	for (SIFTParticleMap::const_iterator p = pm.begin(); 
	     p != pm.end();
	     ++p ) {

	   // ParticleTrajactory t = pms.getTrajactory( i, p->first );
	    pms.getTrajactory( i, p->first );
	    // draw here
	    SIFTParticle::List plist = t.getList();
	    for (SIFTParticle::List::const_iterator ip = plist.begin();
	         ip != plist.end()-1;
		 ++ip ) {

		// get feature point ip
		cvLine( output_frame, 
			cvPoint(ip->x, ip->y), 
			cvPoint((ip+1)->x, (ip+1)->y),
		        CV_RGB(255, 0, 0), 1, CV_AA );
	    }
	    pms.remove( i, p->first );
	}
    }
*/ 
    cvSaveImage( output_file1, output_frame1 );
    cvSaveImage( output_file2, output_frame2 );
    cvReleaseImage( &output_frame1 );
    cvReleaseImage( &output_frame2 );
    cvReleaseImage( &frame );
}
