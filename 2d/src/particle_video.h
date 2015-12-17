#ifndef H_PARTICLE_VIDEO
#define H_PARTICLE_VIDEO

#include <vector>
#include <map>
#include <iostream>

#include <cxcore.h>

namespace ParticleVideo {
    using std::vector;
    using std::map;


    typedef unsigned int ParticleIdx;

    template<class ParticleType>
	class Particle {
	    public:
		typedef map<ParticleIdx, ParticleType> Map;
		typedef vector<ParticleType> List;

		Particle(const ParticleType &p) : _p(p) {}

		const ParticleType &getFeature() const { return _p; }

	    private:
		ParticleType _p;
	};

    template <class ParticleType>
	class Trajactory { // {{{
	    public:
		Trajactory( const int init_frame ) : _init_frame(init_frame) {}
		Trajactory( std::istream &in ) {
		    in >> *this;
		}

		unsigned int length() const { return _plist.size(); }

		void add( const ParticleType &p ) {
		    _plist.push_back(p);
		}

		int begin() const { return _init_frame; }

		typename Particle<ParticleType>::List getList() const { return _plist; }

		friend std::istream &operator >> (
			std::istream &stream,
			Trajactory<ParticleType> &t) {

		    int length = 0;
		    stream >> t._init_frame >> length;
		    //					std::cerr << t._init_frame << std::endl;
		    //					std::cerr << length << std::endl;
		    for (int i = 0; i < length; ++i) {
			ParticleType p;
			stream >> p;
			t._plist.push_back(p);
		    }
		    return stream;
		}

	    private:
		int _init_frame;
		typename Particle<ParticleType>::List _plist;;
		// }}}
	};

    template <class ParticleType>
	std::ostream &operator << (
		std::ostream &stream,
		const Trajactory<ParticleType> &t) {

	    stream << t.begin() << " " << t.length() << " ";
	    typename Particle<ParticleType>::List plist = t.getList();
	    for (
		    typename Particle<ParticleType>::List::const_iterator p = plist.begin();
		    p != plist.end();
		    ++p) {
		stream << *p << " ";
	    }
	    return stream;
	}

    template<class ParticleType>
	class ParticleMaps :
	    public vector< typename Particle<ParticleType>::Map > {
		public:
		    ParticleMaps() {
			_storage = cvCreateMemStorage(0);
			_g =
			    cvCreateGraph(
				    CV_SEQ_KIND_GRAPH,
				    sizeof(CvGraph),
				    sizeof(CvGraphVtx),
				    sizeof(CvGraphEdge),
				    _storage);
		    }

		    ~ParticleMaps() {
			cvReleaseMemStorage(&_storage);
		    }

		    void addLink(ParticleIdx pt1, ParticleIdx pt2) {
			cvGraphAddEdge( _g, pt1, pt2 );
		    }

		    bool updateLinkWeights(const unsigned int frame_num) { // {{{
			const int SIGMA_L = 1.5;

			if (frame_num >= this->size())
			    return false;

			const typename Particle<ParticleType>::Map &pm =
			    (*this)[frame_num];		

			for (
				typename Particle<ParticleType>::Map::const_iterator p =
				pm.begin();
				p != pm.end();
				++p) {

			    const ParticleIdx particle_num = p->first;

			    std::vector<ParticleIdx> nbs =
				getFrameNeighbors( frame_num, particle_num );
			    Trajactory<ParticleType> t = getTrajactory(frame_num, p->first);

			    for (
				    std::vector<ParticleIdx>::const_iterator nb = nbs.begin();
				    nb != nbs.end();
				    ++nb) { 
				CvGraphEdge *edge = cvFindGraphEdge( _g, particle_num, *nb );

				// calculate weight {{{
				if (edge != NULL) { 
				    int cnt = 0;
				    double cost = 0;
				    double weight_sum = 0;

				    {
					CvPoint2D32f pt1 = getPosition( frame_num, particle_num );
					CvPoint2D32f pt2 = getPosition( frame_num, *nb );

					unsigned int cur_frame_num = frame_num-1;

					while (
						exist( cur_frame_num, *nb ) &&
						exist( cur_frame_num, particle_num) ) {

					    const CvPoint2D32f npt1 =
						getPosition( cur_frame_num, particle_num );
					    const CvPoint2D32f npt2 =
						getPosition( cur_frame_num, *nb);

					    const double u1 = pt1.x - npt1.x;
					    const double v1 = pt1.y - npt1.y;
					    const double u2 = pt2.x - npt2.x;
					    const double v2 = pt2.y - npt2.y;

					    double weight = exp(-((signed)cur_frame_num-(signed)frame_num)*((signed)cur_frame_num-(signed)frame_num));
					    cost += weight*((u1-u2)*(u1-u2) + (v1-v2)*(v1-v2));

					    weight_sum += weight;
					    pt1 = npt1;
					    pt2 = npt2;
					    ++cnt;
					    --cur_frame_num;
					}
				    }

				    {
					CvPoint2D32f pt1 = getPosition( frame_num, particle_num );
					CvPoint2D32f pt2 = getPosition( frame_num, *nb );

					unsigned int cur_frame_num = frame_num+1;
					while (
						exist( cur_frame_num, *nb ) &&
						exist( cur_frame_num, particle_num) ) {

					    const CvPoint2D32f npt1 =
						getPosition( cur_frame_num, particle_num );
					    const CvPoint2D32f npt2 =
						getPosition( cur_frame_num, *nb);

					    const double u1 = pt1.x - npt1.x;
					    const double v1 = pt1.y - npt1.y;
					    const double u2 = pt2.x - npt2.x;
					    const double v2 = pt2.y - npt2.y;

					    double weight = exp(-((signed)cur_frame_num-(signed)frame_num)*((signed)cur_frame_num-(signed)frame_num));
					    cost += weight*((u1-u2)*(u1-u2) + (v1-v2)*(v1-v2));
					    weight_sum += weight;

					    pt1 = npt1;
					    pt2 = npt2;
					    ++cnt;
					    ++cur_frame_num;
					} 
				    }

				    //                  std::cerr << "( " << weight_sum << ", " << cnt << " )" << std::endl;
				    edge->weight = cnt == 0 ? 0 :
					exp(-cost*cost/weight_sum/weight_sum / (SIGMA_L*SIGMA_L));
				    if (edge->weight < 0.01)
					cvGraphRemoveEdge(_g, particle_num, *nb);
				} // }}}
			    }
			}

			return true;
			// }}}
		    }

		    bool isLinked(
			    const ParticleIdx particle_num1, 
			    const ParticleIdx particle_num2) const { // {{{
			return cvFindGraphEdge( _g, particle_num1, particle_num2 ) != NULL;
			// }}}
		    }

		    double getLinkWeight(
			    const ParticleIdx particle_num1, 
			    const ParticleIdx particle_num2) const { // {{{
			CvGraphEdge *edge =
			    cvFindGraphEdge( _g, particle_num1, particle_num2 );
			return edge->weight;
			// }}}
		    }

		    Trajactory<ParticleType> getTrajactory(
			    const unsigned int frame_num,
			    const ParticleIdx particle_num) const {
			int cur_frame_num = frame_num;

			while ( exist(cur_frame_num, particle_num) ) {
			    --cur_frame_num;
			}

			cur_frame_num++;
			Trajactory<ParticleType> t(cur_frame_num);

			while ( exist(cur_frame_num, particle_num) ) {
			    t.add( getParticle(cur_frame_num, particle_num ) );
			    ++cur_frame_num;
			}

			return t;	
		    }

		    void removeForward( const unsigned int frame_num, const ParticleIdx particle_num ) {
			int cur_frame_num = frame_num;
			while( exist(cur_frame_num, particle_num) ) {
			    (*this)[cur_frame_num].erase(particle_num);
			    ++cur_frame_num;
			}

			if ( !exist(frame_num-1, particle_num)) {
			    cvGraphRemoveVtx( _g, particle_num );
			}
		    }

		    void remove( const unsigned int frame_num, const ParticleIdx particle_num ) {
			int cur_frame_num = frame_num;
			while( exist(cur_frame_num, particle_num) ) {
			    (*this)[cur_frame_num].erase(particle_num);
			    --cur_frame_num;
			}

			cur_frame_num = frame_num+1;
			while( exist(cur_frame_num, particle_num) ) {
			    (*this)[cur_frame_num].erase(particle_num);
			    ++cur_frame_num;
			}

			cvGraphRemoveVtx( _g, particle_num );
		    }

		    ParticleType getParticle (
			    const unsigned int frame_num,
			    const ParticleIdx particle_num ) const {
			if (
				frame_num >= this->size() ||
				(*this)[frame_num].find(particle_num) == (*this)[frame_num].end()) 
			    return ParticleType();

			const ParticleType &p = (*this)[frame_num].at(particle_num);
			return p;
		    }

		    CvPoint2D32f getPosition(
			    const unsigned int frame_num,
			    const ParticleIdx particle_num ) const {
			if (
				frame_num >= this->size() ||
				(*this)[frame_num].find(particle_num) == (*this)[frame_num].end()) 
			    return cvPoint2D32f(0, 0);

			const ParticleType &p = (*this)[frame_num].at(particle_num);
			return cvPoint2D32f(p.x, p.y);
		    }

		    vector<ParticleIdx> getFrameNeighbors(
			    const unsigned int frame_num, const ParticleIdx particle_num) const { // {{{
			vector<ParticleIdx> nbs;
			CvGraphVtx *v = cvGetGraphVtx( _g, particle_num );
			CvGraphEdge *edge = v->first;
			const typename Particle<ParticleType>::Map &pm = (*this)[frame_num];
			while (edge) {
			    CvGraphVtx *v2 =
				edge->vtx[ edge->vtx[0] != v ? 0 : 1 ];
			    int idx_v2 = cvGraphVtxIdx( _g, v2 );
			    if ( pm.find(idx_v2) != pm.end() ) {
				nbs.push_back( idx_v2 );
			    }

			    edge = CV_NEXT_GRAPH_EDGE( edge, v );
			}
			return nbs;
			// }}}
		    };

		    ParticleIdx add(
			    const unsigned int frame_num,
			    const ParticleType &p) {
			if (frame_num >= this->size()) {
			    this->resize(frame_num+1);
			}

			const ParticleIdx vertex_idx =
			    cvGraphAddVtx( _g );
			(*this)[frame_num].insert( std::make_pair(vertex_idx, p) );
			return vertex_idx;
		    }

		    unsigned int guaranteeSize(
			    const unsigned int size) {
			if (this->size() < size) {
			    this->resize(size);
			}
			return this->size();
		    }

		    bool extend(
			    const unsigned int frame_num,
			    const unsigned int particle_num,
			    const ParticleType& p) { // {{{

			if (frame_num >= this->size() ) {
			    this->resize(frame_num+1);
			}

			(*this)[frame_num][particle_num] = p;
			return true;
			// }}}
		    }

		    bool find(ParticleIdx &idx, const unsigned int frame_num, const CvPoint2D32f pt) const {
			if (frame_num >= this->size()) return false;
			const typename Particle<ParticleType>::Map &pm =
			    (*this).at(frame_num);

			for (
				typename Particle<ParticleType>::Map::const_iterator p = pm.begin();
				p != pm.end();
				++p) {
			    if ((float)p->second.x == pt.x && (float)p->second.y == pt.y) {
				idx = p->first;
				return true;
			    }
			}
			return false;
		    }

		    bool exist(
			    const unsigned int frame_num,
			    const ParticleIdx idx) const {
			if (frame_num >= this->size()) return false;
			return (*this)[frame_num].find(idx) != (*this)[frame_num].end();
		    }

		    bool exist(const unsigned int frame_num, const CvPoint2D32f pt)
			const {
			    if (frame_num >= this->size()) return false;
			    const typename Particle<ParticleType>::Map &pm = (*this).at(frame_num);
			    for (
				    typename Particle<ParticleType>::Map::const_iterator p = pm.begin();
				    p != pm.end();
				    ++p) {
				if (p->second.x  == pt.x && p->second.y == pt.y) {
				    return true;
				}
			    }
			    return false;
			}

		    unsigned int numOfParticles(
			    const int frame_num) const {
			return (*this)[frame_num].size(); 
		    }

		private:
		    CvMemStorage *_storage;
		    CvGraph *_g;
	    };
};

#endif
