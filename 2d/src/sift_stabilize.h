#include "sift_feature.h"
#include "sift_particle_video.h"

typedef ParticleVideo::ParticleMaps<SIFT::FeaturePoint> SIFTParticleMaps;
typedef ParticleVideo::Particle<SIFT::FeaturePoint>::Map SIFTParticleMap;
typedef ParticleVideo::Particle<SIFT::FeaturePoint>::List ParticleList;

class SimilarityTransformMatrix {
    public:
	static const double DELTA_SCALE;
	SimilarityTransformMatrix() : a(1), b(0), dx(0), dy(0),
	sv(sqrt(1-DELTA_SCALE)) {}

	double a;
	double b;
	double dx;
	double dy;
	double sv;
};

const double SimilarityTransformMatrix::DELTA_SCALE = 1e-3;

void load_tracks(SIFTParticleMaps &pms, const char *filename) {
    using namespace SIFT;
    using namespace ParticleVideo;

    std::ifstream in(filename);
    while (true) {
	Trajactory<FeaturePoint> t(in);
	if (in.eof())
	    break;

	if (t.length() > 3) {
	    ParticleList  plist = t.getList();
	    ParticleIdx idx = pms.add(t.begin(), plist[0]);
	    for (unsigned int i = 1; i < plist.size(); ++i) {
		pms.extend(t.begin()+i, idx, plist[i]);
	    }
	}
    }
    in.close();
}

