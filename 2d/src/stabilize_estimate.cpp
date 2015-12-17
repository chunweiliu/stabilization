/* Scale, Rotation, Translation, constrained on uncovered area, and scaling */
/* Trying different weighting. */

#include <gsl/gsl_multifit_nlin.h>

#include <limits>
#include <highgui.h>

#include <fstream>

#include "sift_feature.h"
#include "sift_particle_video.h"
#include "sift_stabilize.h"
#include "cv.h"

//#define BUG

#define DISPARITY_ASYMMETRIC

using namespace SIFT;
using namespace ParticleVideo;
using std::cout;
using std::endl;

struct ParamT {
    ParamT() : begin_frame(0), end_frame(0) {}
    ParamT(
	    const vector< vector<ParticleIdx> > *filtered_particles1, 
	    const vector< vector<ParticleIdx> > *filtered_particles2, 
	    const SIFTParticleMaps *pms1,
	    const SIFTParticleMaps *pms2,
	    const FeatureListLoader *feature_list_loader1,
	    const FeatureListLoader *feature_list_loader2,	
	    //const char* key_dir1,
	    //const char* key_dir2,
	    //const char* image_name,
	    vector<SimilarityTransformMatrix> *tList1,
	    vector<SimilarityTransformMatrix> *tList2,
	    const unsigned int begin_frame,
	    const unsigned int end_frame) :
	filtered_particles1(filtered_particles1),
	filtered_particles2(filtered_particles2),
	pms1(pms1),
	pms2(pms2),
	feature_list_loader1(feature_list_loader1),
	feature_list_loader2(feature_list_loader2),
	//key_dir1(key_dir1),
	//key_dir2(key_dir2),
	//image_name(image_name),
	tList1(tList1),
	tList2(tList2),
	begin_frame(begin_frame),
	end_frame(end_frame) {}

    const vector< vector<ParticleIdx> > *filtered_particles1;
    const vector< vector<ParticleIdx> > *filtered_particles2;
    const SIFTParticleMaps *pms1;
    const SIFTParticleMaps *pms2;
    const FeatureListLoader *feature_list_loader1;
    const FeatureListLoader *feature_list_loader2;
    //const char * key_dir1;
    //const char * key_dir2;
    //const char * image_name;
    vector<SimilarityTransformMatrix> *tList1; 
    vector<SimilarityTransformMatrix> *tList2; 
    const unsigned int begin_frame;
    const unsigned int end_frame;
};

std::map<std::pair<int, ParticleIdx>, double> WeightMap;
static const unsigned int MIN_FILTERED = 5;
static const unsigned int FILTERED = 30;
static const double DELTA_REG = 1e-6;
static const double SCALE_TOR = 1e-1;
static const double SIGMA_SMOOTH = 10;

static const double LAMBDA_SMOOTH = 0;
static double LAMBDA_SCALE_REGUR;
static double LAMBDA_UNCOVERED;

static double LAMBDA_DISPARITY;
static double LAMBDA_ASYMMETRIC;
//static const double LAMBDA1 = 1;
//static const double LAMBDA_SCALE_REGUR= LAMBDA1 * 1e3;
//static const double LAMBDA_UNCOVERED = LAMBDA1 * 1e3;

static const unsigned int NUM_OPTIMIZED_ITERATION = 15;

static const unsigned int NUM_FEATURE_MATCHING_LIMIT = 50;

static const double MARGIN_RATIO = 0;
static int FRAME_WIDTH;
static int FRAME_HEIGHT;


static double uncovered_cost(
	const SimilarityTransformMatrix &m, 
	const int w,
	const int h) {
    double v = 0;
    CvPoint2D32f p[4] = {
	{ 0, 0 },
	{ FRAME_WIDTH-1, 0},
	{ 0, FRAME_HEIGHT-1},
	{ FRAME_WIDTH-1, FRAME_HEIGHT-1}};

    const double s = sqrt(m.a*m.a+m.b*m.b);
    const double cos = m.a/s;
    const double sin = m.b/s;
    const double dx  = m.dx/s;
    const double dy  = m.dy/s;

    for (int k = 0; k < 4; ++k) {
	const double ox = p[k].x;
	const double oy = p[k].y;
	const double x = (cos*ox + sin*oy)/s - (cos*dx+sin*dy);
	const double y = (-sin*ox + cos*oy)/s - (-sin*dx+cos*dy);
	const double mid_x = (FRAME_WIDTH-1)/2.;
	const double mid_y = (FRAME_HEIGHT-1)/2.;
	const double w = (FRAME_WIDTH)/2.;
	const double h = (FRAME_HEIGHT)/2.;
	const double ex = fabs(x-mid_x) - w < 0 ? 0 : fabs(x-mid_x) - w;
	const double ey = fabs(y-mid_y) - h < 0 ? 0 : fabs(y-mid_y) - h;
	v +=
	    FRAME_HEIGHT*(sqrt(ex*ex+DELTA_REG*DELTA_REG) - DELTA_REG) +
	    FRAME_WIDTH*(sqrt(ey*ey+DELTA_REG*DELTA_REG) - DELTA_REG);
    }
    return v/FRAME_WIDTH/FRAME_HEIGHT;
}

void initSimilarityTransformMatrix( const gsl_vector * x, 
	const unsigned int begin_frame,
	const int window_width, 
	vector<SimilarityTransformMatrix> &tList, 
	const int p_offset ) {

    for (unsigned int i = 0; i < window_width; ++i) {

	SimilarityTransformMatrix &t = tList[begin_frame+i];
	const double theta =     gsl_vector_get(x, p_offset + 4*i+2);
	const double scale_var = gsl_vector_get(x, p_offset + 4*i+3);
	const double scale =
	    sqrt(scale_var*scale_var+SimilarityTransformMatrix::DELTA_SCALE);

	t.sv = scale_var;

	t.dx = scale * gsl_vector_get(x, p_offset + 4*i);
	t.dy = scale * gsl_vector_get(x, p_offset + 4*i+1);

	t.a  = scale*cos(theta);
	t.b  = scale*sin(theta);

    }

}

void fillE_roughnessTerm( gsl_vector * f,
	const unsigned int window_width, 
	const unsigned int begin_frame,
	vector<SimilarityTransformMatrix> &tList, 
	const SIFTParticleMaps &pms, 
	const vector<vector<ParticleIdx> > &filtered_particles,
	const unsigned int n_offset )
{

    for (int i = -1; i <= (signed) window_width; ++i) {
	const int f0 = begin_frame+i-1;
	const int f1 = begin_frame+i;
	const int f2 = begin_frame+i+1;

	if (f0 >= 0 && f2 < (signed)pms.size()) {
	    const double dx0 = tList[f0].dx;
	    const double dx1 = tList[f1].dx;
	    const double dx2 = tList[f2].dx;

	    const double dy0 = tList[f0].dy;
	    const double dy1 = tList[f1].dy;
	    const double dy2 = tList[f2].dy;

	    const double a0 = tList[f0].a;
	    const double a1 = tList[f1].a;
	    const double a2 = tList[f2].a;

	    const double b0 = tList[f0].b;
	    const double b1 = tList[f1].b;
	    const double b2 = tList[f2].b;

	    const double sv1 = tList[f1].sv;
	    const double s1 = sqrt(sv1*sv1+SimilarityTransformMatrix::DELTA_SCALE);

	    if ( filtered_particles[f1].size() >= MIN_FILTERED) {
		for (unsigned int j = 0; j < filtered_particles[f1].size(); ++j) {
		    const ParticleIdx particle_num = filtered_particles[f1][j];

		    const CvPoint2D32f p0 = pms.getPosition(f0, particle_num);
		    const CvPoint2D32f p1 = pms.getPosition(f1, particle_num);
		    const CvPoint2D32f p2 = pms.getPosition(f2, particle_num);

		    const double ax =
			(    a0*p0.x - b0*p0.y + dx0)
			-2*( a1*p1.x - b1*p1.y + dx1)
			+ (  a2*p2.x - b2*p2.y + dx2);

		    const double ay =
			(    b0*p0.x + a0*p0.y + dy0)
			-2*( b1*p1.x + a1*p1.y + dy1)
			+ (  b2*p2.x + a2*p2.y + dy2);

		    int tidx =  FILTERED*(i+1) + j;

		    double weight = sqrt(WeightMap[std::make_pair(f1, particle_num)]);
		    gsl_vector_set( f, n_offset + 2*tidx+0, weight*ax/s1 );
		    gsl_vector_set( f, n_offset + 2*tidx+1, weight*ay/s1 );
		}
	    } else {
		const CvPoint2D32f pt[4] = {
		    {0, 0},
		    {FRAME_WIDTH, 0},
		    {0, FRAME_HEIGHT},
		    {FRAME_WIDTH, FRAME_HEIGHT} };
		for (unsigned int k = 0; k < 4; ++k) {
		    const CvPoint2D32f p0 = pt[k];
		    const CvPoint2D32f p1 = pt[k];
		    const CvPoint2D32f p2 = pt[k];

		    const double ax =
			(    a0*p0.x - b0*p0.y + dx0)
			-2*( a1*p1.x - b1*p1.y + dx1)
			+ (  a2*p2.x - b2*p2.y + dx2);

		    const double ay =
			(    b0*p0.x + a0*p0.y + dy0)
			-2*( b1*p1.x + a1*p1.y + dy1)
			+ (  b2*p2.x + a2*p2.y + dy2);

		    int tidx =  FILTERED*(i+1) + k;

		    double weight = sqrt(1./4);
		    gsl_vector_set( f, n_offset + 2*tidx, weight*ax/s1 );
		    gsl_vector_set( f, n_offset + 2*tidx+1, weight*ay/s1 );
		}
	    }
	}
    }
}

void fillE_degradationTerm( gsl_vector * f, 
	const unsigned int window_width, 
	const unsigned int begin_frame,
	vector<SimilarityTransformMatrix> &tList, 
	const unsigned int n_offset ) 
{
    int tidx = 2*FILTERED*(window_width+1);

    for (unsigned int i = 0; i < window_width; ++i) {
	const SimilarityTransformMatrix &m =
	    tList[begin_frame+i];

	const double scale = sqrt(m.a*m.a+m.b*m.b);
	if (scale > 1 ) 
	    gsl_vector_set( f, n_offset + tidx+i,
		    LAMBDA_SCALE_REGUR*(scale-1)*(scale-1));

	gsl_vector_set( f, n_offset + tidx+window_width+i,
		LAMBDA_UNCOVERED*uncovered_cost(m, FRAME_WIDTH, FRAME_HEIGHT));
    }
}

/* 
   Write down the rule of f(x, a) with varables x and parameters a
   And use gsl_vector_set to assign these values
 */

int s_cost_f (const gsl_vector * x, void * params, gsl_vector * f) { 
    ParamT *p = (ParamT *) params;

    // Initialize parameters
    const vector<vector<ParticleIdx> > &filtered_particles1 =
	*(p->filtered_particles1);
    const vector<vector<ParticleIdx> > &filtered_particles2 =
	*(p->filtered_particles2);

    const SIFTParticleMaps &pms1 =
	*(p->pms1);
    const SIFTParticleMaps &pms2 =
	*(p->pms2);

    const FeatureListLoader &feature_list_loader1 =
	*(p->feature_list_loader1);
    const FeatureListLoader &feature_list_loader2 = 
	*(p->feature_list_loader2);
    
    const unsigned int begin_frame = p->begin_frame;
    const unsigned int end_frame   = p->end_frame;
    const unsigned int window_width = end_frame-begin_frame+1;

    const unsigned int n_offset = 2*FILTERED*(window_width+1) + window_width + window_width;
    const unsigned int n_offset2= 2*n_offset;
    const unsigned int p_offset = 4*window_width;

    vector<SimilarityTransformMatrix> &tList1 = *(p->tList1);
    vector<SimilarityTransformMatrix> &tList2 = *(p->tList2);
/*
    initSimilarityTransformMatrix( x, window_width, begin_frame, tList, 0 );
    initSimilarityTransformMatrix( x, window_width, begin_frame, tList2, p_offset );
    
    std::cout << " List1 ------------------------------------------- " << std::endl;
    for (unsigned int i = 0; i < window_width; ++i) {

	SimilarityTransformMatrix &t = tList[begin_frame+i];
	std::cout << t.sv << " " << t.dx << " " << t.dy << " " << t.a << " " << t.b << std::endl;
    }
    std::cout << " List2 ------------------------------------------- " << std::endl;
    for (unsigned int i = 0; i < window_width; ++i) {

	SimilarityTransformMatrix &t = tList2[begin_frame+i];
	std::cout << t.sv << " " << t.dx << " " << t.dy << " " << t.a << " " << t.b << std::endl;
    }
    
    std::cout << "List1' ------------------------------------------- " << std::endl;
    */
    for (unsigned int i = 0; i < window_width; ++i) {

	SimilarityTransformMatrix &t = tList1[begin_frame+i];

	const double theta = gsl_vector_get(x, 4*i+2);
	const double scale_var = gsl_vector_get(x, 4*i+3);
	const double scale =
	    sqrt(scale_var*scale_var+SimilarityTransformMatrix::DELTA_SCALE);

	t.sv = scale_var;

	t.dx = scale * gsl_vector_get(x, 4*i);
	t.dy = scale * gsl_vector_get(x, 4*i+1);

	t.a  = scale*cos(theta);
	t.b  = scale*sin(theta);
//	std::cout << t.sv << " " << t.dx << " " << t.dy << " " << t.a << " " << t.b << std::endl;
    }
//  std::cout << "List2' ------------------------------------------- " << std::endl;
    for (unsigned int i = 0; i < window_width; ++i) {
	SimilarityTransformMatrix &t = tList2[begin_frame+i];

	const double theta = gsl_vector_get(x, p_offset + 4*i+2);
	const double scale_var = gsl_vector_get(x, p_offset + 4*i+3);
	const double scale =
	    sqrt(scale_var*scale_var+SimilarityTransformMatrix::DELTA_SCALE);

	t.sv = scale_var;

	t.dx = scale * gsl_vector_get(x, p_offset + 4*i);
	t.dy = scale * gsl_vector_get(x, p_offset + 4*i+1);

	t.a  = scale*cos(theta);
	t.b  = scale*sin(theta);
//	std::cout << t.sv << " " << t.dx << " " << t.dy << " " << t.a << " " << t.b << std::endl;
    }

    gsl_vector_set_all(f, 0);

    fillE_roughnessTerm( f, window_width, begin_frame, tList1, pms1, filtered_particles1, 0 );
    fillE_roughnessTerm( f, window_width, begin_frame, tList2, pms2, filtered_particles2, n_offset );

    fillE_degradationTerm( f, window_width, begin_frame, tList1, 0 ); 
    fillE_degradationTerm( f, window_width, begin_frame, tList2, n_offset ); 

#ifdef DISPARITY_ASYMMETRIC
    // E_disparity(T, T') + E_asymmetric(T, T')
    for (unsigned int i = 0; i < window_width; ++i) {

	const SimilarityTransformMatrix &m1 = tList1[begin_frame+i];
	const SimilarityTransformMatrix &m2 = tList2[begin_frame+i];
	
	const double dx1 = m1.dx;
	const double dx2 = m2.dx;

	const double dy1 = m1.dy;
	const double dy2 = m2.dy;

	const double a1 = m1.a;
	const double a2 = m2.a;

	const double b1 = m1.b;
	const double b2 = m2.b;

	FeatureList feature_list1;
	FeatureList feature_list2;

	feature_list_loader1.load_raw( feature_list1, begin_frame+i );
	feature_list_loader2.load_raw( feature_list2, begin_frame+i );

	const unsigned int num_feature_matching = feature_list1.size() > NUM_FEATURE_MATCHING_LIMIT ? NUM_FEATURE_MATCHING_LIMIT : feature_list1.size() ;
	for (unsigned int j = 0; j < num_feature_matching; ++j) {

	    unsigned int fidx = n_offset2 + 2*(i*NUM_FEATURE_MATCHING_LIMIT + j); // for x, and y
	    
	    FeaturePoint p1 = feature_list1.at(j);
	    FeaturePoint p2 = feature_list2.at(j);

	    const double disparity_x = ((a1*p1.x - b1*p1.y + dx1) - (a2*p2.x - b2*p2.y + dx2))
		                       -(p1.x - p2.x);
	    const double disparity_y = ((b1*p1.x + a1*p1.y + dy1) - (b2*p2.x + a2*p2.y + dy2));

	    gsl_vector_set( f, fidx  , LAMBDA_DISPARITY *disparity_x );
	    gsl_vector_set( f, fidx+1, LAMBDA_ASYMMETRIC*disparity_y );

	}

    }

#endif

    // Show optimization cost function
    {
	double sum11 = 0;
	double sum12 = 0;
	double sum13 = 0;
	for (unsigned int i = 0; i < 2*FILTERED*(window_width+1); ++i)
	    sum11 += gsl_vector_get( f, i ) * gsl_vector_get(f, i);
	for (unsigned int i = 2*FILTERED*(window_width+1); i < 2*FILTERED*(window_width+1)+window_width; ++i)
	    sum12 += gsl_vector_get( f, i ) * gsl_vector_get(f, i);
	for (unsigned int i = 2*FILTERED*(window_width+1)+window_width;
		i < 2*FILTERED*(window_width+1)+2*window_width; ++i)
	    sum13 += gsl_vector_get( f, i ) * gsl_vector_get(f, i);

	std::cout << "L: Roughnesss: " << sum11 << std::endl;
	std::cout << "L: Scaling: " << sum12 << std::endl;
	std::cout << "L: Uncovered: " << sum13 << std::endl;
	std::cout << "L: Cost: " << sum11 + sum12 + sum13 << std::endl;


	double sum21 = 0;
	double sum22 = 0;
	double sum23 = 0;    
	for (unsigned int i = n_offset; i < n_offset + 2*FILTERED*(window_width+1); ++i)
	    sum21 += gsl_vector_get( f, i ) * gsl_vector_get(f, i);
	for (unsigned int i = n_offset + 2*FILTERED*(window_width+1);
		i < n_offset + 2*FILTERED*(window_width+1)+window_width; ++i)
	    sum22 += gsl_vector_get( f, i ) * gsl_vector_get(f, i);
	for (unsigned int i = n_offset + 2*FILTERED*(window_width+1)+window_width;
		i < n_offset + 2*FILTERED*(window_width+1)+2*window_width; ++i)
	    sum23 += gsl_vector_get( f, i ) * gsl_vector_get(f, i);

	std::cout << "R: Roughnesss: " << sum21 << std::endl;
	std::cout << "R: Scaling: " << sum22 << std::endl;
	std::cout << "R: Uncovered: " << sum23 << std::endl;
	std::cout << "R: Cost: " << sum21 + sum22 + sum23 << std::endl;


#ifdef DISPARITY_ASYMMETRIC
	double sum31 = 0;
	double sum32 = 0;
	for (unsigned int i = n_offset2; i < n_offset2 + 2*window_width*NUM_FEATURE_MATCHING_LIMIT; i+=2) {
	    sum31 += gsl_vector_get( f, i ) * gsl_vector_get( f, i );
	    sum32 += gsl_vector_get( f, i+1 ) * gsl_vector_get( f, i+1 );
	}

	std::cout << "Disparity: " << sum31 << std::endl;
	std::cout << "Asymmetric: " << sum32 << std::endl << std::endl;

	std::cout << "Total Cost: " 
	    << sum11+sum12+sum13+
	       sum21+sum22+sum23+
	       sum31+sum32 << std::endl;
	std::cout << "----------" << std::endl << std::endl;
#endif
    }

    return GSL_SUCCESS;
}

void fillE_roughnessJacobianTerm ( gsl_matrix * J, 
	const unsigned int window_width,
	const unsigned int begin_frame,
	vector<SimilarityTransformMatrix> &tList, 
	const SIFTParticleMaps &pms, 
	const vector<vector<ParticleIdx> > &filtered_particles,
	const unsigned int p_offset,
	const unsigned int n_offset )
{
    for (int i = -1; i <= (signed)window_width; ++i) {
	const int i0 = i-1;
	const int i1 = i;
	const int i2 = i+1;

	const int f0 = begin_frame+i-1;
	const int f1 = begin_frame+i;
	const int f2 = begin_frame+i+1;

	if (f0 >= 0 && f2 < (signed) pms.size()) {
	    const double dx0 = tList[f0].dx;
	    const double dx2 = tList[f2].dx;

	    const double dy0 = tList[f0].dy;
	    const double dy2 = tList[f2].dy;

	    const double a0 = tList[f0].a;
	    const double a1 = tList[f1].a;
	    const double a2 = tList[f2].a;

	    const double b0 = tList[f0].b;
	    const double b1 = tList[f1].b;
	    const double b2 = tList[f2].b;

	    const int ix0 = 4*i0;
	    const int ix1 = 4*i1;
	    const int ix2 = 4*i2;

	    const int iy0 = 4*i0+1;
	    const int iy1 = 4*i1+1;
	    const int iy2 = 4*i2+1;

	    const int it0 = 4*i0+2;
	    const int it1 = 4*i1+2;
	    const int it2 = 4*i2+2;

	    const int is0 = 4*i0+3;
	    const int is1 = 4*i1+3;
	    const int is2 = 4*i2+3;

	    const double sv0 = tList[f0].sv;
	    const double sv1 = tList[f1].sv;
	    const double sv2 = tList[f2].sv;

	    const double s0 = sqrt(sv0*sv0+SimilarityTransformMatrix::DELTA_SCALE);
	    const double s1 = sqrt(sv1*sv1+SimilarityTransformMatrix::DELTA_SCALE);
	    const double s2 = sqrt(sv2*sv2+SimilarityTransformMatrix::DELTA_SCALE);

	    if ( filtered_particles[f1].size() >= MIN_FILTERED) {
		for (unsigned int j = 0; j < filtered_particles[f1].size(); ++j) {
		    const ParticleIdx particle_num = filtered_particles[f1][j];

		    const CvPoint2D32f p0 = pms.getPosition(f0, particle_num);
		    const CvPoint2D32f p1 = pms.getPosition(f1, particle_num);
		    const CvPoint2D32f p2 = pms.getPosition(f2, particle_num);

		    unsigned int fidx = FILTERED*(i+1)+j;

		    double weight = sqrt(WeightMap[std::make_pair(f1, particle_num)]);

		    if (i0 >= 0) {
			gsl_matrix_set(J, n_offset + 2*fidx,   p_offset + ix0, s0/s1*weight);
			gsl_matrix_set(J, n_offset + 2*fidx+1, p_offset + ix0, 0);

			gsl_matrix_set(J, n_offset + 2*fidx,   p_offset + iy0, 0);
			gsl_matrix_set(J, n_offset + 2*fidx+1, p_offset + iy0, s0/s1*weight);

			gsl_matrix_set(J, n_offset + 2*fidx,   p_offset + it0, (-b0*p0.x-a0*p0.y)/s1*weight); // what is /s1*weight means?
			gsl_matrix_set(J, n_offset + 2*fidx+1, p_offset + it0, (a0*p0.x-b0*p0.y)/s1*weight);

			gsl_matrix_set(J, n_offset + 2*fidx,   p_offset + is0, (a0*p0.x-b0*p0.y+dx0)/s0*sv0/s0/s1*weight);
			gsl_matrix_set(J, n_offset + 2*fidx+1, p_offset + is0, (b0*p0.x+a0*p0.y+dy0)/s0*sv0/s0/s1*weight);
		    }


		    if (i2 < (signed) window_width) {
			gsl_matrix_set(J, n_offset + 2*fidx,   p_offset + ix2, s2/s1*weight);
			gsl_matrix_set(J, n_offset + 2*fidx+1, p_offset + ix2, 0);

			gsl_matrix_set(J, n_offset + 2*fidx,   p_offset + iy2, 0);
			gsl_matrix_set(J, n_offset + 2*fidx+1, p_offset + iy2, s2/s1*weight);

			gsl_matrix_set(J, n_offset + 2*fidx,   p_offset + it2, (-b2*p2.x-a2*p2.y)/s1*weight);
			gsl_matrix_set(J, n_offset + 2*fidx+1, p_offset + it2, (a2*p2.x-b2*p2.y)/s1*weight);

			gsl_matrix_set(J, n_offset + 2*fidx,   p_offset + is2, (a2*p2.x-b2*p2.y+dx2)/s2*sv2/s2/s1*weight);
			gsl_matrix_set(J, n_offset + 2*fidx+1, p_offset + is2, (b2*p2.x+a2*p2.y+dy2)/s2*sv2/s2/s1*weight);
		    }

		    if (i1 >= 0 && i1 < (signed) window_width) {
			gsl_matrix_set(J, n_offset + 2*fidx,   p_offset + ix1, -2*weight);
			gsl_matrix_set(J, n_offset + 2*fidx+1, p_offset + ix1, 0);

			gsl_matrix_set(J, n_offset + 2*fidx,   p_offset + iy1, 0);
			gsl_matrix_set(J, n_offset + 2*fidx+1, p_offset + iy1, -2*weight);

			gsl_matrix_set(J, n_offset + 2*fidx,   p_offset + it1, -2*(-b1*p1.x-a1*p1.y)/s1*weight);
			gsl_matrix_set(J, n_offset + 2*fidx+1, p_offset + it1, -2*(a1*p1.x-b1*p1.y)/s1*weight);

			gsl_matrix_set(J, n_offset + 2*fidx,   p_offset + is1, 
				-(a0*p0.x-b0*p0.y+dx0 + a2*p2.x-b2*p2.y+dx2)*sv1/s1/s1/s1*weight);
			gsl_matrix_set(J, n_offset + 2*fidx+1, p_offset + is1, 
				-(b0*p0.x+a0*p0.y+dy0 + b2*p2.x+a2*p2.y+dy2)*sv1/s1/s1/s1*weight);
		    }
		}
	    } else {
		const CvPoint2D32f pt[4] = {
		    {0, 0},
		    {FRAME_WIDTH, 0},
		    {0, FRAME_HEIGHT},
		    {FRAME_WIDTH, FRAME_HEIGHT} };
		for (unsigned int k = 0; k < 4; ++k) {
		    const CvPoint2D32f p0 = pt[k];
		    const CvPoint2D32f p1 = pt[k];
		    const CvPoint2D32f p2 = pt[k];

		    unsigned int fidx = FILTERED*(i+1)+k;

		    double weight = sqrt(1./4);

		    if (i0 >= 0) {
			gsl_matrix_set(J, n_offset + 2*fidx,   p_offset + ix0, s0/s1*weight);
			gsl_matrix_set(J, n_offset + 2*fidx+1, p_offset + ix0, 0);

			gsl_matrix_set(J, n_offset + 2*fidx,   p_offset + iy0, 0);
			gsl_matrix_set(J, n_offset + 2*fidx+1, p_offset + iy0, s0/s1*weight);

			gsl_matrix_set(J, n_offset + 2*fidx,   p_offset + it0, (-b0*p0.x-a0*p0.y)/s1*weight);
			gsl_matrix_set(J, n_offset + 2*fidx+1, p_offset + it0, (a0*p0.x-b0*p0.y)/s1*weight);

			gsl_matrix_set(J, n_offset + 2*fidx,   p_offset + is0, (a0*p0.x-b0*p0.y+dx0)/s0*sv0/s0/s1*weight);
			gsl_matrix_set(J, n_offset + 2*fidx+1, p_offset + is0, (b0*p0.x+a0*p0.y+dy0)/s0*sv0/s0/s1*weight);
		    }


		    if (i2 < (signed) window_width) {
			gsl_matrix_set(J, n_offset + 2*fidx,   p_offset + ix2, s2/s1*weight);
			gsl_matrix_set(J, n_offset + 2*fidx+1, p_offset + ix2, 0);

			gsl_matrix_set(J, n_offset + 2*fidx,   p_offset + iy2, 0);
			gsl_matrix_set(J, n_offset + 2*fidx+1, p_offset + iy2, s2/s1*weight);

			gsl_matrix_set(J, n_offset + 2*fidx,   p_offset + it2, (-b2*p2.x-a2*p2.y)/s1*weight);
			gsl_matrix_set(J, n_offset + 2*fidx+1, p_offset + it2, (a2*p2.x-b2*p2.y)/s1*weight);

			gsl_matrix_set(J, n_offset + 2*fidx,   p_offset + is2, (a2*p2.x-b2*p2.y+dx2)/s2*sv2/s2/s1*weight);
			gsl_matrix_set(J, n_offset + 2*fidx+1, p_offset + is2, (b2*p2.x+a2*p2.y+dy2)/s2*sv2/s2/s1*weight);
		    }

		    if (i1 >= 0 && i1 < (signed) window_width) {
			gsl_matrix_set(J, n_offset + 2*fidx,   p_offset + ix1, -2*weight);
			gsl_matrix_set(J, n_offset + 2*fidx+1, p_offset + ix1, 0);

			gsl_matrix_set(J, n_offset + 2*fidx,   p_offset + iy1, 0);
			gsl_matrix_set(J, n_offset + 2*fidx+1, p_offset + iy1, -2*weight);

			gsl_matrix_set(J, n_offset + 2*fidx,   p_offset + it1, -2*(-b1*p1.x-a1*p1.y)/s1*weight);
			gsl_matrix_set(J, n_offset + 2*fidx+1, p_offset + it1, -2*(a1*p1.x-b1*p1.y)/s1*weight);

			gsl_matrix_set(J, n_offset + 2*fidx,   p_offset + is1, 
				-(a0*p0.x-b0*p0.y+dx0 + a2*p2.x-b2*p2.y+dx2)*sv1/s1/s1/s1*weight);
			gsl_matrix_set(J, n_offset + 2*fidx+1, p_offset + is1, 
				-(b0*p0.x+a0*p0.y+dy0 + b2*p2.x+a2*p2.y+dy2)*sv1/s1/s1/s1*weight);
		    }
		}
	    }
	}
    }

}
void fillE_degradationJacobian( gsl_matrix * J, 
	const unsigned int window_width,
	const unsigned int begin_frame,
	vector<SimilarityTransformMatrix> &tList, 
	const unsigned int p_offset,
	const unsigned int n_offset )
{
	int tidx = 2*FILTERED*(window_width+1);
	for (unsigned int i = 0; i < window_width; ++i) {
	    const int jx = 4*i;
	    const int jy = 4*i+1;
	    const int jt = 4*i+2;
	    const int js = 4*i+3;

	    const SimilarityTransformMatrix &m = tList[begin_frame+i];
	    const double sv = m.sv;
	    const double scale =
		sqrt(sv*sv+SimilarityTransformMatrix::DELTA_SCALE);

	    if (scale > 1)
		gsl_matrix_set( J, n_offset + tidx+i, p_offset + js, LAMBDA_SCALE_REGUR * 2 * (scale-1) * sv /scale); 

	    {
		CvPoint2D32f p[4] = {
		    { 0, 0 },
		    { FRAME_WIDTH-1, 0},
		    { 0, FRAME_HEIGHT-1},
		    { FRAME_WIDTH-1, FRAME_HEIGHT-1}};

		{
		    double sum_x = 0;
		    double sum_y = 0;
		    double sum_t = 0;
		    double sum_s = 0;
		    const double s = sqrt(m.a*m.a+m.b*m.b);
		    const double cos = m.a/s;
		    const double sin = m.b/s;
		    const double dx  = m.dx/s;
		    const double dy  = m.dy/s;

		    const double mid_x = (FRAME_WIDTH-1)/2.;
		    const double mid_y = (FRAME_HEIGHT-1)/2.;
		    const double w = (FRAME_WIDTH)/2.;
		    const double h = (FRAME_HEIGHT)/2.;

		    for (int k = 0; k < 4; ++k) {
			const double ox = p[k].x;
			const double oy = p[k].y;
			const double x = (cos*ox + sin*oy)/s - (cos*dx+sin*dy);
			const double y = (-sin*ox + cos*oy)/s - (-sin*dx+cos*dy);

			const double ex = fabs(x-mid_x) - w < 0 ? 0 : fabs(x-mid_x) - w;
			const double ey = fabs(y-mid_y) - h < 0 ? 0 : fabs(y-mid_y) - h;

			const double sign_x = x > mid_x ? 1 : -1;
			const double sign_y = y > mid_y ? 1 : -1;

			const double cx = sqrt(ex*ex+DELTA_REG*DELTA_REG) * FRAME_WIDTH;
			const double cy = sqrt(ey*ey+DELTA_REG*DELTA_REG) * FRAME_HEIGHT;

			sum_x += ex*(sign_x*(-cos))/cx + ey*sign_y*sin/cy;
			sum_y += ex*(sign_x*(-sin))/cx + ey*sign_y*(-cos)/cy;
			sum_t +=
			    ex*sign_x*((-sin*ox+cos*oy)/s-(-sin*dx+cos*dy))/cx +
			    ey*sign_y*((-cos*ox-sin*oy)/s-(-cos*dx-sin*dy))/cy;
			sum_s += 
			    ex*sign_x*(-(cos*ox+sin*oy)/s/s*sv/s)/cx +
			    ey*sign_y*(-(-sin*ox+cos*oy)/s/s*sv/s)/cy;
		    }

		    gsl_matrix_set( J, n_offset + tidx+window_width+i, p_offset + jx, LAMBDA_UNCOVERED*sum_x);
		    gsl_matrix_set( J, n_offset + tidx+window_width+i, p_offset + jy, LAMBDA_UNCOVERED*sum_y);
		    gsl_matrix_set( J, n_offset + tidx+window_width+i, p_offset + jt, LAMBDA_UNCOVERED*sum_t);
		    gsl_matrix_set( J, n_offset + tidx+window_width+i, p_offset + js, LAMBDA_UNCOVERED*sum_s);
		}
	    }
	}

}
int s_cost_df (const gsl_vector * x, void *params, gsl_matrix * J) {
    ParamT *p = (ParamT *) params;

    const vector<vector<ParticleIdx> > &filtered_particles1 =
	*(p->filtered_particles1);

    const SIFTParticleMaps &pms1 =
	*(p->pms1);

    const vector<vector<ParticleIdx> > &filtered_particles2 =
	*(p->filtered_particles2);

    const SIFTParticleMaps &pms2 =
	*(p->pms2);

    const FeatureListLoader &feature_list_loader1 =
	*(p->feature_list_loader1);
    const FeatureListLoader &feature_list_loader2 = 
	*(p->feature_list_loader2);

    const unsigned int begin_frame = p->begin_frame;
    const unsigned int end_frame   = p->end_frame;
    const unsigned int window_width = end_frame - begin_frame+1;

    const int n_offset = 2*FILTERED*(window_width+1) + window_width + window_width;
    const int n_offset2 = 2*n_offset;
    const int p_offset = 4 * window_width;

    vector<SimilarityTransformMatrix> &tList1 = *(p->tList1);
    vector<SimilarityTransformMatrix> &tList2 = *(p->tList2);


    //initSimilarityTransformMatrix( x, window_width, begin_frame, tList,  0 );
    //initSimilarityTransformMatrix( x, window_width, begin_frame, tList2, p_offset );
    
    for (unsigned int i = 0; i < window_width; ++i) {

	SimilarityTransformMatrix &t = tList1[begin_frame+i];

	const double theta = gsl_vector_get(x, 4*i+2);
	const double scale_var = gsl_vector_get(x, 4*i+3);
	const double scale =
	    sqrt(scale_var*scale_var+SimilarityTransformMatrix::DELTA_SCALE);

	t.sv = scale_var;

	t.dx = scale * gsl_vector_get(x, 4*i);
	t.dy = scale * gsl_vector_get(x, 4*i+1);

	t.a  = scale*cos(theta);
	t.b  = scale*sin(theta);
    }
    for (unsigned int i = 0; i < window_width; ++i) {
	SimilarityTransformMatrix &t = tList2[begin_frame+i];

	const double theta = gsl_vector_get(x, p_offset + 4*i+2);
	const double scale_var = gsl_vector_get(x, p_offset + 4*i+3);
	const double scale =
	    sqrt(scale_var*scale_var+SimilarityTransformMatrix::DELTA_SCALE);

	t.sv = scale_var;

	t.dx = scale * gsl_vector_get(x, p_offset + 4*i);
	t.dy = scale * gsl_vector_get(x, p_offset + 4*i+1);

	t.a  = scale*cos(theta);
	t.b  = scale*sin(theta);
    }

    

    gsl_matrix_set_all(J, 0);

    fillE_roughnessJacobianTerm ( J, window_width, begin_frame,
	tList1, 
	pms1, 
	filtered_particles1,
	0,
	0);
    fillE_roughnessJacobianTerm ( J, window_width, begin_frame,
	tList2, 
	pms2, 
	filtered_particles2,
	p_offset,
	n_offset);
    
    fillE_degradationJacobian( J, window_width, begin_frame,
	tList1, 0, 0 );
    fillE_degradationJacobian( J, window_width, begin_frame,
	tList2, p_offset, n_offset );

#ifdef DISPARITY_ASYMMETRIC
    // Jocobian matrix setting for E_disparity(T, T') and E_asymmetric(T, T')
    for (unsigned int i = 0; i < window_width; ++i) {

	const unsigned int jx = 4*i;
	const unsigned int jy = 4*i+1;
	const unsigned int jt = 4*i+2;
	const unsigned int js = 4*i+3;

	const SimilarityTransformMatrix &m1 = tList1[begin_frame+i];
	const SimilarityTransformMatrix &m2 = tList2[begin_frame+i];
	
	const double dx1 = m1.dx;
	const double dx2 = m2.dx;

	const double dy1 = m1.dy;
	const double dy2 = m2.dy;

	const double a1 = m1.a;
	const double a2 = m2.a;

	const double b1 = m1.b;
	const double b2 = m2.b;

	const double sv1 = m1.sv;
	const double sv2 = m2.sv;

	const double s1 = sqrt(sv1*sv1+SimilarityTransformMatrix::DELTA_SCALE);
	const double s2 = sqrt(sv2*sv2+SimilarityTransformMatrix::DELTA_SCALE);

	FeatureList feature_list1;
	FeatureList feature_list2;

	feature_list_loader1.load_raw( feature_list1, begin_frame+i );
	feature_list_loader2.load_raw( feature_list2, begin_frame+i );

	const unsigned int num_feature_matching = feature_list1.size() > NUM_FEATURE_MATCHING_LIMIT ? NUM_FEATURE_MATCHING_LIMIT : feature_list1.size() ;
	for (unsigned int j = 0; j < num_feature_matching; ++j) {

	    const unsigned int fidx = n_offset2 + 2*(i*NUM_FEATURE_MATCHING_LIMIT + j); // for x, and y
	    
	    FeaturePoint p1 = feature_list1.at(j);
	    FeaturePoint p2 = feature_list2.at(j);

	    gsl_matrix_set( J, fidx,   jx, LAMBDA_DISPARITY  * 1 );
	    gsl_matrix_set( J, fidx+1, jx, LAMBDA_ASYMMETRIC * 0 );
	    
	    gsl_matrix_set( J, fidx,   jy, LAMBDA_DISPARITY  * 0 );
	    gsl_matrix_set( J, fidx+1, jy, LAMBDA_ASYMMETRIC * 1 );

	    gsl_matrix_set( J, fidx,   jt, LAMBDA_DISPARITY  * (-b1*p1.x - a1*p1.y) );
	    gsl_matrix_set( J, fidx+1, jt, LAMBDA_ASYMMETRIC * ( a1*p1.x - b1*p1.y) );

	    gsl_matrix_set( J, fidx,   js, LAMBDA_DISPARITY  * (-b1*p1.x - a1*p1.y)/s1 );
	    gsl_matrix_set( J, fidx,   js, LAMBDA_ASYMMETRIC * ( a1*p1.x - b1*p1.y)/s1 );
	    
	    gsl_matrix_set( J, fidx,   p_offset + jx, LAMBDA_DISPARITY  * -1 );
	    gsl_matrix_set( J, fidx+1, p_offset + jx, LAMBDA_ASYMMETRIC * 0 );
	    
	    gsl_matrix_set( J, fidx,   p_offset + jy, LAMBDA_DISPARITY  * 0 );
	    gsl_matrix_set( J, fidx+1, p_offset + jy, LAMBDA_ASYMMETRIC * -1 );

	    gsl_matrix_set( J, fidx,   p_offset + jt, LAMBDA_DISPARITY  * -(-b2*p2.x - a2*p2.y) );
	    gsl_matrix_set( J, fidx+1, p_offset + jt, LAMBDA_ASYMMETRIC * -( a2*p2.x - b2*p2.y) );

	    gsl_matrix_set( J, fidx,   p_offset + js, LAMBDA_DISPARITY  * -(-b2*p2.x - a2*p2.y)/s2 );
	    gsl_matrix_set( J, fidx,   p_offset + js, LAMBDA_ASYMMETRIC * -( a2*p2.x - b2*p2.y)/s2 );
	    
	
	}

    }
#endif
    return GSL_SUCCESS;
}

int s_cost_fdf(const gsl_vector * x, void *data, gsl_vector * f, gsl_matrix * J) {
    /* 
varables:
x is a p x 1 vector 

observation:
f is a n x 1 vector
J is a n x p matrix
     */
    s_cost_f(x, data, f);
    s_cost_df(x, data, J);
    return GSL_SUCCESS;
}


double optimizeSimilarityTransfomation(
	vector<SimilarityTransformMatrix> &tList1, 
	vector<SimilarityTransformMatrix> &tList2,
	vector< vector<ParticleIdx> > &filtered_particles1, 
	vector< vector<ParticleIdx> > &filtered_particles2,
	const SIFTParticleMaps &pms1,
	const SIFTParticleMaps &pms2,
	const FeatureListLoader &feature_list_loader1,
	const FeatureListLoader &feature_list_loader2,
	unsigned int begin_frame,
	unsigned int end_frame ) {

    unsigned int window_width = end_frame-begin_frame+1;

    ParamT param(&filtered_particles1, &filtered_particles2, &pms1, &pms2, &feature_list_loader1, &feature_list_loader2, &tList1, &tList2, begin_frame, end_frame);

#ifdef BUG
    std::cout << "998 ParamT" << std::endl;
#endif
    // Use orignal one, see if we can solve this 
    //int num_feature_matching = countFeatureMatching( key_dir1 ); 

#ifdef DISPARITY_ASYMMETRIC
    const int n = 2*(
	    2 * FILTERED * (window_width+1) +   // roughness 
	    window_width +                     // scaling
	    window_width )                    // uncovered
	+ window_width*NUM_FEATURE_MATCHING_LIMIT
	+ window_width*NUM_FEATURE_MATCHING_LIMIT;
#else
    const int n = 2* (
	    2 * FILTERED * (window_width+1) +   // roughness 
	    window_width +                     // scaling
	    window_width );                   // uncovered
#endif

#ifdef BUG
    std::cout << "1011 n = " << n  << std::endl;
#endif
    //const int n = 2 * (2*FILTERED*(window_width+1)); // roughness left + right

    const int p = 4 * window_width + 
	4 * window_width ; // left and right similarity transform

#ifdef BUG
    std::cout << "1018 p = " << p  << std::endl;
#endif
    //const int n = 2 * (2*FILTERED*(window_width+1)); // roughness left + right
    int n_offset = 2 * FILTERED * (window_width+1) + window_width + window_width;  
    int p_offset = 4 * window_width; // index to right similarity transform

#ifdef BUG
    std::cout << "1025 n_offset = " << n_offset << std::endl;
    std::cout << "1026 p_offset = " << p_offset << std::endl;
#endif

    gsl_multifit_function_fdf cost_function;
    {
	cost_function.n	  = n; // #functions
	cost_function.f	  = &s_cost_f;
	cost_function.df  = &s_cost_df;
	cost_function.fdf     = &s_cost_fdf;
	cost_function.p       = p; // #indepdent_varable
	cost_function.params  = (void *)&param;
    }

#ifdef BUG
    std::cout << "1040 gsl_multifit_function_fdf" << std::endl;
#endif
    const gsl_multifit_fdfsolver_type *T =
	gsl_multifit_fdfsolver_lmsder;

    gsl_vector *x =
	gsl_vector_alloc( p );

#ifdef BUG
    std::cout << "1049 gsl_vector_alloc( p )" << std::endl;
#endif
    // Initalize varables 4 * 2 * window_width
    for (unsigned int i = 0; i < window_width; ++i) {
	SimilarityTransformMatrix &t = tList1[begin_frame+i];
	const double scale = sqrt(t.a*t.a+t.b*t.b);

	gsl_vector_set(x, 4*i, t.dx / scale);
	gsl_vector_set(x, 4*i+1, t.dy / scale);
	gsl_vector_set(x, 4*i+2, atan2(t.b, t.a));
	gsl_vector_set(x, 4*i+3, t.sv);
    }
    for (unsigned int i = 0; i < window_width; ++i) {
	SimilarityTransformMatrix &t = tList2[begin_frame+i];
	const double scale = sqrt(t.a*t.a+t.b*t.b);

	gsl_vector_set(x, p_offset + 4*i, t.dx / scale);
	gsl_vector_set(x, p_offset + 4*i+1, t.dy / scale);
	gsl_vector_set(x, p_offset + 4*i+2, atan2(t.b, t.a));
	gsl_vector_set(x, p_offset + 4*i+3, t.sv);
    }

#ifdef BUG
    std::cout << "1072 gsl_vector_set" << std::endl;
#endif
    gsl_multifit_fdfsolver *s =
	gsl_multifit_fdfsolver_alloc(T, n, p);

#ifdef BUG
    std::cout << "1078 gsl_multifit_fdfsolver" << std::endl;
#endif



    // has a bug in this line: fixed
    gsl_multifit_fdfsolver_set(s, &cost_function, x);

#ifdef BUG
    std::cout << "1087 gsl_multifit_fdfsolver_set" << std::endl;
#endif
    size_t iter = 0;
    int status;

    do {
	iter++;
	std::cout << "Iteration: #" << iter << std::endl;
	status = gsl_multifit_fdfsolver_iterate(s);
	if (status) break;

	status = gsl_multifit_test_delta(s->dx, s->x, window_width*1e-3,
		window_width*1e-3);

	if (status == GSL_SUCCESS) printf ("Minimum found at:\n"); 
    }
    //while (status == GSL_CONTINUE && iter < 15); 
    while (status == GSL_CONTINUE && iter < NUM_OPTIMIZED_ITERATION); 
    std::cout << std::endl;

    // Assign final solution to tList1 and tList2 
    for (unsigned int i = 0; i < window_width; ++i) {
	const double theta =
	    gsl_vector_get( s->x, 4*i+2 );

	const double scale_var =
	    gsl_vector_get( s->x, 4*i+3 );

	const double scale =
	    sqrt(scale_var*scale_var+SimilarityTransformMatrix::DELTA_SCALE);

	tList1[begin_frame+i].a = scale*cos(theta);
	tList1[begin_frame+i].b = scale*sin(theta);

	tList1[begin_frame+i].dx = scale*gsl_vector_get(s->x, 4*i);
	tList1[begin_frame+i].dy = scale*gsl_vector_get(s->x, 4*i+1);

	tList1[begin_frame+i].sv = scale_var;
    }
    for (unsigned int i = 0; i < window_width; ++i) {
	const double theta =
	    gsl_vector_get( s->x, p_offset + 4*i+2 );

	const double scale_var =
	    gsl_vector_get( s->x, p_offset + 4*i+3 );

	const double scale =
	    sqrt(scale_var*scale_var+SimilarityTransformMatrix::DELTA_SCALE);

	tList2[begin_frame+i].a = scale*cos(theta);
	tList2[begin_frame+i].b = scale*sin(theta);

	tList2[begin_frame+i].dx = scale*gsl_vector_get(s->x, p_offset + 4*i);
	tList2[begin_frame+i].dy = scale*gsl_vector_get(s->x, p_offset + 4*i+1);

	tList2[begin_frame+i].sv = scale_var;
    }

    double result = 0;
    //for (unsigned int i = 0; i < 2*FILTERED*(window_width+1)+2*window_width; ++i)
    //for (unsigned int i = 0; i < 2*2*FILTERED*(window_width+1)+2*window_width; ++i)
    // Calculating fianl energy function cost
    for (unsigned int i = 0; i < n; ++i)
	result += gsl_vector_get( s->f, i ) * gsl_vector_get(s->f, i);

    std::cout << std::endl;
    gsl_multifit_fdfsolver_free(s); // L: solver from GUN for the objective function
    gsl_vector_free(x);
    return result;
}

const int WINDOW_SIZE = 60;
const int WINDOW_STEP = WINDOW_SIZE/2;

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
	    std::cout << frame_num  << " has only " << filtered_particles[frame_num].size() << " tracks." <<
		std::endl;
	} 
    }
    // }}}
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

int main( int argc, char *argv[] ) {
    if (argc != 11) {
	std::cerr << "Usage: " << argv[0]
		  << " <input_key_dir1> <input_key_dir2>"
		  << " <input_tracks_file1> <input_tracks_file2>"
		  << " <output_path_file1> <output_path_file2>"
		  << " <lambda_scale_regur> <lambda_uncovered>"
		  << " <lambda_disparity> <lambda_asymmetric>"
		  << std::endl;
	return -1;
    }

    const char *input_key_dir1 = argv[1];
    const char *input_key_dir2 = argv[2];
    const char *tracks_filename1 = argv[3];
    const char *tracks_filename2 = argv[4];
    const char *output_filename1 = argv[5];
    const char *output_filename2 = argv[6];
    const char *lambda_scale_regur = argv[7];
    const char *lambda_uncovered = argv[8];
    const char *lambda_disparity = argv[9];
    const char *lambda_asymmetric = argv[10];

    LAMBDA_SCALE_REGUR = atof( lambda_scale_regur );
    LAMBDA_UNCOVERED = atof( lambda_uncovered );
    LAMBDA_DISPARITY = atof( lambda_disparity );
    LAMBDA_ASYMMETRIC = atof( lambda_asymmetric );


    // Get frame width and height for computation 
    /*
       char image_postfix;
       char image_name[64];
       parseFileName( argv[1], image_name, image_postfix );

       char image_file[64];
       sprintf( image_file, "%s%s_0000_%c.png", input_image_dir1, image_name, image_postfix );
       IplImage *frame = cvLoadImage( image_file );
       FRAME_WIDTH = frame->width;
       FRAME_HEIGHT = frame->height;
       cvReleaseImage( &frame );
     */
    // XXX: 
    {
	FRAME_WIDTH = 640;
	FRAME_HEIGHT= 480;
    }
    // 

#ifndef NDEBUG
    cout << "Input output filename1: " << output_filename1 << endl;
    cout << "Frame width: " << FRAME_WIDTH << endl;
    cout << "Frame heght: " << FRAME_HEIGHT<< endl;
#endif

    char image_postfix;
    char image_name[128];
    parseFileName( argv[1], image_name, image_postfix );

    // Loading both left(1) and right(2) trajectories
    SIFTParticleMaps pms1; // Using SIFT as ParticleType
    SIFTParticleMaps pms2; // Using SIFT as ParticleType

    load_tracks(pms1, tracks_filename1);
    load_tracks(pms2, tracks_filename2);
    vector< vector<ParticleIdx> > filtered_particles1(pms1.size());
    filtered_tracks(filtered_particles1, pms1);
    //std::cout << filtered_particles1[1].size() << std::endl;

    vector< vector<ParticleIdx> > filtered_particles2(pms2.size());
    filtered_tracks(filtered_particles2, pms2);

    vector<SimilarityTransformMatrix> tList1(pms1.size()); // The varables in our system
    vector<SimilarityTransformMatrix> tList2(pms2.size()); // The varables in our system

    FeatureListLoader feature_list_loader1( input_key_dir1, image_name, 'l' );
    FeatureListLoader feature_list_loader2( input_key_dir2, image_name, 'r' );

    for (int iw = 0; iw < ( (signed int)pms1.size()-WINDOW_SIZE+WINDOW_STEP-1)/WINDOW_STEP+1; ++iw) {
	const unsigned int begin_frame = iw*WINDOW_STEP+1;
	unsigned int end_frame = begin_frame+WINDOW_SIZE-1;
	if (end_frame > pms1.size()-2) {
	    end_frame = pms1.size()-2;
	}

	//Dout( dc::notice, "From " << begin_frame << " to " << end_frame);
	std::cout << "From " << begin_frame << " to " << end_frame << std::endl;

	// Initalization of varables lastList
	double last = std::numeric_limits<double>::max();
	vector<SimilarityTransformMatrix> lastList1;
	for (unsigned int i = begin_frame; i <= end_frame; ++i) {
	    const double sv = 1 + SCALE_TOR;
	    tList1[i].sv = sv;

	    const double scale =
		sqrt(sv*sv+SimilarityTransformMatrix::DELTA_SCALE);
	    tList1[i].a = scale;
	    tList1[i].b = 0;
	    tList1[i].dx = -(scale-1)/2*FRAME_WIDTH;
	    tList1[i].dy = -(scale-1)/2*FRAME_HEIGHT;
	}
	vector<SimilarityTransformMatrix> lastList2;
	for (unsigned int i = begin_frame; i <= end_frame; ++i) {
	    const double sv = 1 + SCALE_TOR;
	    tList2[i].sv = sv;

	    const double scale =
		sqrt(sv*sv+SimilarityTransformMatrix::DELTA_SCALE);
	    tList2[i].a = scale;
	    tList2[i].b = 0;
	    tList2[i].dx = -(scale-1)/2*FRAME_WIDTH;
	    tList2[i].dy = -(scale-1)/2*FRAME_HEIGHT;
	}

	if (SCALE_TOR > 0.5) {
	    while (1) {
		double result = optimizeSimilarityTransfomation(
			tList1, tList2, 
			filtered_particles1, filtered_particles2, 
			pms1, pms2,
			feature_list_loader1, feature_list_loader2,
			//input_key_dir1, input_key_dir2, image_name,
			begin_frame, end_frame);
		if (result >= last)
		    break;

		last = result;
		lastList1 = tList1;
		lastList2 = tList2;

		for (unsigned int i = begin_frame; i <= end_frame; ++i) {
		    const double oscale = sqrt(tList1[i].a*tList1[i].a+tList1[i].b*tList1[i].b);
		    const double sv    = tList1[i].sv * 0.99;
		    tList1[i].sv = sv;

		    const double scale =
			sqrt(sv*sv+SimilarityTransformMatrix::DELTA_SCALE);
		    tList1[i].a = tList1[i].a / oscale * scale;
		    tList1[i].b = tList1[i].b / oscale * scale;
		    tList1[i].dx = tList1[i].dx / oscale * scale;
		    tList1[i].dy = tList1[i].dy / oscale * scale;
		}
		for (unsigned int i = begin_frame; i <= end_frame; ++i) {
		    const double oscale = sqrt(tList2[i].a*tList2[i].a+tList2[i].b*tList2[i].b);
		    const double sv    = tList2[i].sv * 0.99;
		    tList2[i].sv = sv;

		    const double scale =
			sqrt(sv*sv+SimilarityTransformMatrix::DELTA_SCALE);
		    tList2[i].a = tList2[i].a / oscale * scale;
		    tList2[i].b = tList2[i].b / oscale * scale;
		    tList2[i].dx = tList2[i].dx / oscale * scale;
		    tList2[i].dy = tList2[i].dy / oscale * scale;
		}

	    }
	    tList1 = lastList1;
	    tList2 = lastList2;
	} else {
#ifdef BUG
	    std::cout << "1432 initalize tList" << std::endl;
#endif
	    optimizeSimilarityTransfomation(
		    tList1, tList2,
		    filtered_particles1, filtered_particles2,
		    pms1, pms2,
		    feature_list_loader1, feature_list_loader2,
		    //input_key_dir1, input_key_dir2, image_name,
		    begin_frame, end_frame);
	}
    }

    std::ofstream fp1(output_filename1);
    for (unsigned int i = 0; i < pms1.size(); ++i) {
	fp1 << tList1[i].a << " " << tList1[i].b << " " << tList1[i].dx << " " <<
	    tList1[i].dy << std::endl;
    }
    fp1.close();
    std::ofstream fp2(output_filename2);
    for (unsigned int i = 0; i < pms2.size(); ++i) {
	fp2 << tList2[i].a << " " << tList2[i].b << " " << tList2[i].dx << " " <<
	    tList2[i].dy << std::endl;
    }
    fp2.close();
    return 0;

}

