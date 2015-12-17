

#include <vector>
#include <string>
#include <fstream>

#include <cv.h>
#include <highgui.h>

#include "particle_video.h"
#include "sift_feature.h"

#define NDEBUG
#define USE_ANN
#ifdef USE_ANN
#include "ANN.h"
#endif

#ifndef NDEBUG
using std::cerr;
using std::endl;
#endif

using std::vector;
using std::map;
using ParticleVideo::ParticleMaps;
using ParticleVideo::Particle;
using ParticleVideo::ParticleIdx;
using ParticleVideo::Trajactory;
using SIFT::FeaturePoint;
using SIFT::FeatureList;
using SIFT::FeatureListLoader;

typedef ParticleMaps<FeaturePoint>		SIFTParticleMaps;
typedef Particle<FeaturePoint>::Map		SIFTParticleMap;
typedef Trajactory<FeaturePoint>	ParticleTrajactory;

static void addParticles(
	SIFTParticleMaps &pms,
	const unsigned int frame_num,
	const FeatureList &fl) {

    std::cerr << "Adding particles..." << std::endl;
    pms.guaranteeSize( frame_num+1 );
    {
	int added = 0;
	//int loop_num = 0; //debug
	for (
		FeatureList::const_iterator p = fl.begin();
		p != fl.end();
		++p ) {
	    //std::cerr << loop_num << " loops..." << std::endl; //debug
	    //loop_num++; // debug
	    if (! pms.exist( frame_num, cvPoint2D32f( p->x, p->y ) ) ) {
		pms.add( frame_num, *p );
		++added;
	    }
	}
	std::cerr << added << " particles added..." << std::endl;
    }
}

#ifdef USE_ANN
static void buildTree(
	ANNpointArray &dataPts, ANNkd_tree *&tree, const FeatureList &fl) {

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

#else
static void buildTree(
	CvMat **dataPts, CvFeatureTree **tree, const FeatureList &fl) {

    const unsigned int dim = FeaturePoint::NUM_OF_DIMENSION;
    const int nPts = fl.size();

    *dataPts = cvCreateMat( nPts, dim, CV_32FC1 );

    for (unsigned int i = 0; i < fl.size(); ++i) {
	for (unsigned int j = 0; j < dim; ++j) {
	    cvSetReal2D( *dataPts, i, j, fl[i].desc[j] );
	}
    }

    *tree = cvCreateFeatureTree( *dataPts );
}
#endif

static unsigned int propagation(
	SIFTParticleMaps &pms,
	const unsigned int prev_frame_num,
	const unsigned int frame_num,
	const FeatureList &prev_fl,
	const FeatureList &fl) {

    const unsigned int dim = FeaturePoint::NUM_OF_DIMENSION;

#ifdef USE_ANN 
    if (prev_fl.size() < 5 || fl.size() < 5) {
	return 0;
    }
    ANNpointArray dataPts1;
    ANNpointArray dataPts2;

    ANNkd_tree *tree1;
    ANNkd_tree *tree2;

    buildTree( dataPts1, tree1, prev_fl );
    buildTree( dataPts2, tree2, fl );

    ANNdist dist;
    ANNidx idx1, idx2;

    int matched = 0;
    for (unsigned int i = 0; i < prev_fl.size(); ++i) {
	tree2->annkSearch( dataPts1[i], 1, &idx2, &dist, 0 );
	if (dist < dim*dim*2*2) {
	    tree1->annkSearch( dataPts2[idx2], 1, &idx1, &dist, 0 );
	    if (idx1 == (signed) i) {
		ParticleIdx pIdx = 0;
		if (
			pms.find(
			    pIdx,
			    prev_frame_num,
			    cvPoint2D32f(prev_fl[idx1].x, prev_fl[idx1].y))) {
		    pms.extend( frame_num, pIdx, fl[idx2] );

		} else {
		    pms.add( prev_frame_num, prev_fl[idx1] );
		    pms.add( frame_num, fl[idx2] );
		}

		++matched;
	    }
	}
    }
    std::cerr << matched << " features matched..." << std::endl;

    annDeallocPts(dataPts1);
    annDeallocPts(dataPts2);
    delete tree1;
    delete tree2;

#else
    CvMat *dataPts1= NULL;
    CvMat *dataPts2= NULL;

    CvFeatureTree *tree1 = NULL;
    CvFeatureTree *tree2 = NULL;

    buildTree( &dataPts1, &tree1, prev_fl );
    buildTree( &dataPts2, &tree2, fl );

    CvMat *result1 = cvCreateMat( dataPts1->rows, 1, CV_32SC1 );
    CvMat *result2 = cvCreateMat( dataPts2->rows, 1, CV_32SC1 );

    CvMat *dist1   = cvCreateMat( dataPts1->rows, 1, CV_64FC1 );
    CvMat *dist2   = cvCreateMat( dataPts2->rows, 1, CV_64FC1 );

    cvFindFeatures( tree1, dataPts2, result2, dist2, 1, 100);
    cvFindFeatures( tree2, dataPts1, result1, dist1, 1, 100);

    int matched = 0;
    for (int i = 0; i < dataPts1->rows; ++i) {
	int idx2 = cvGetReal2D( result1, i, 0 );
	int idx1 = cvGetReal2D( result2, idx2, 0 );
	if (
		cvGetReal2D( dist1, i, 0 ) < dim*2 &&
		idx1 == i ) {
	    cvLine(
		    img,
		    cvPoint(prev_fl[i].x, prev_fl[i].y),
		    cvPoint(fl[idx2].x, fl[idx2].y),
		    CV_RGB(255, 0, 0));
	    ++matched;
	}
    }

    std::cerr << matched << " features matched... " << std::endl;

    cvReleaseMat( &dist1 );
    cvReleaseMat( &dist2 );

    cvReleaseMat( &result1 );
    cvReleaseMat( &result2 );

    cvReleaseFeatureTree( tree1 );
    cvReleaseFeatureTree( tree2 );

    cvReleaseMat( &dataPts1 );
    cvReleaseMat( &dataPts2 );

#endif

    return matched;
}

static CvSubdiv2D* init_delaunay( CvMemStorage* storage, CvRect rect ) {
    CvSubdiv2D* subdiv;
    subdiv = cvCreateSubdiv2D(
	    CV_SEQ_KIND_SUBDIV2D, sizeof(*subdiv),
	    sizeof(CvSubdiv2DPoint),
	    sizeof(CvQuadEdge2D),
	    storage
	    );

    cvInitSubdivDelaunay2D( subdiv, rect );
    return subdiv;
}


bool operator < (const CvPoint2D32f& a, const CvPoint2D32f& b) {
    return (a.x == b.x ? a.y < b.y : a.x < b.x);
}   

static void link(
	SIFTParticleMaps &pms,
	const unsigned int frame_num,
	const IplImage *img) {
#ifndef NDEBUG
    cerr << "Linking..." << endl;

#endif
    CvRect rect = {0, 0, img->width, img->height};
    assert(rect.width > 0 && rect.height > 0);
    CvMemStorage *storage = cvCreateMemStorage(0);
    CvSubdiv2D *subdiv = init_delaunay( storage, rect );

    map<CvPoint2D32f, ParticleIdx> fMap;

    SIFTParticleMap pm = pms[frame_num];
#ifndef NDEBUG
    cerr << "Number of feature matches: " << pm.size() << endl;
#endif
    for (
	    SIFTParticleMap::const_iterator p = pm.begin();
	    p != pm.end();
	    ++p) {
	const FeaturePoint &fp = p->second;
	CvPoint2D32f pt = cvPoint2D32f(fp.x, fp.y);
#ifndef NDEBUG
	cerr << fp.x << " " << fp.y << endl;
#endif
	cvSubdivDelaunay2DInsert( subdiv, pt );
	fMap[pt] = p->first;
    }

    {
	CvSeqReader  reader;
	int total = subdiv->edges->total;
	int elem_size = subdiv->edges->elem_size;

	cvStartReadSeq( (CvSeq*)(subdiv->edges), &reader, 0 );

	for(int i = 0; i < total; i++ ) {
	    CvQuadEdge2D* edge = (CvQuadEdge2D*)(reader.ptr);

	    if( CV_IS_SET_ELEM( edge )) {
		CvSubdiv2DPoint* org_pt = cvSubdiv2DEdgeOrg((CvSubdiv2DEdge)edge);
		CvSubdiv2DPoint* dst_pt = cvSubdiv2DEdgeDst((CvSubdiv2DEdge)edge);
		if( org_pt && dst_pt ) {
		    CvPoint2D32f org = org_pt->pt;
		    CvPoint2D32f dst = dst_pt->pt;
		    if (
			    fMap.find(org) != fMap.end() &&
			    fMap.find(dst) != fMap.end()) {
			pms.addLink( fMap[org], fMap[dst] );
		    }

		}
	    }

	    CV_NEXT_SEQ_ELEM( elem_size, reader );
	}
    }

    cvReleaseMemStorage( &storage );
#ifndef NDEBUG
    cerr << "End of linking..." << endl;
#endif
}

static void pruneParticles(
	SIFTParticleMaps &pms,
	const unsigned int frame_num) {
    SIFTParticleMap pm = pms[frame_num-1];
    vector<ParticleIdx> removeList;
    for (
	    SIFTParticleMap::const_iterator p = pm.begin();
	    p != pm.end();
	    ++p) {
	const ParticleIdx idx = p->first;
	if ( pms.exist( frame_num, idx ) ) {
	    vector<ParticleIdx> nbs = pms.getFrameNeighbors( frame_num-1, idx );
	    const CvPoint2D32f pt11 = pms.getPosition(frame_num-1, idx);
	    const CvPoint2D32f pt12 = pms.getPosition(frame_num, idx);

	    double weight_sum = 0;
	    for (unsigned int i = 0; i < nbs.size(); ++i) {
		const ParticleIdx nIdx = nbs[i];
		if ( pms.exist( frame_num, nIdx ) ) {
		    weight_sum += pms.getLinkWeight(idx, nIdx);
		}
	    }

	    double cost = 0;
	    for (unsigned int i = 0; i < nbs.size(); ++i) {
		const ParticleIdx nIdx = nbs[i];
		if ( pms.exist( frame_num, nIdx ) ) {
		    const CvPoint2D32f pt21 = pms.getPosition(frame_num-1, nIdx);
		    const CvPoint2D32f pt22 = pms.getPosition(frame_num, nIdx);

		    double diff_u = (pt12.x-pt11.x)-(pt22.x-pt21.x);
		    double diff_v = (pt12.y-pt11.y)-(pt22.y-pt21.y);
		    double weight = pms.getLinkWeight(idx, nIdx) / weight_sum;
		    cost += weight * (diff_u*diff_u+diff_v*diff_v);
		}
	    }

	    if (cost > 16 || weight_sum < 1) {
		removeList.push_back(idx);
	    }
	}
    }

    for (unsigned int i = 0; i < removeList.size(); ++i) {
	pms.removeForward( frame_num-1, removeList[i]);
    }
    std::cerr << "Deleted: " << removeList.size() << std::endl;
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
    if (argc != 4) {
	std::cerr << "Usage: " << argv[0]
	    << " <input_image_dir> <input_key_dir> <output_track_file>"
	    << std::endl;
	return -1;
    }

    const char *input_image_dir = argv[1];
    const char *input_key_dir = argv[2];
    const char *output_filename = argv[3];

    char image_postfix;
    char image_name[128];
    parseFileName( argv[1], image_name, image_postfix );
    // {{{
    {
	int frame_num = 0;
	char image_file[128];
	sprintf(image_file, "%s%s_%04d_%c.png", input_image_dir, image_name, frame_num, image_postfix);		
	IplImage *frame = cvLoadImage( image_file );
	IplImage *prev_frame = cvCloneImage( frame );
	FeatureListLoader fll( input_key_dir, image_name, image_postfix );
	FeatureList prev_fl;
	SIFTParticleMaps pms;

	fll.load( prev_fl, 0 );
	frame_num = 1;

	addParticles( pms, 0, prev_fl );
	while (1) {		        
	    char image_file[128];
	    sprintf(image_file, "%s%s_%04d_%c.png", input_image_dir, image_name, frame_num, image_postfix);
	    frame = cvLoadImage(image_file);
	    if (frame == NULL)
		break;

	    printf( "Loading frame %d...\n", frame_num );
	    FeatureList fl;
	    fll.load(fl, frame_num);

	    if (propagation( pms, frame_num-1, frame_num, prev_fl, fl ) > 0) {
		link( pms, frame_num-1, prev_frame );
		link( pms, frame_num, frame );
		pruneParticles( pms, frame_num );
		pms.updateLinkWeights( frame_num );
	    }

	    prev_fl = fl;
	    cvCopy( frame, prev_frame );
	    ++frame_num;
	}

	std::ofstream out(output_filename);
	for (unsigned int i = 0; i < pms.size(); ++i) {
	    SIFTParticleMap pm = pms[i];
	    for (
		    SIFTParticleMap::const_iterator p = pm.begin();
		    p != pm.end();
		    ++p) {
		ParticleTrajactory t = pms.getTrajactory( i, p->first );
		out << t << std::endl;
		pms.remove( i, p->first );
	    }
	}
	out.close();
	cvReleaseImage( &frame );
	cvReleaseImage( &prev_frame );
    } // }}}

}
