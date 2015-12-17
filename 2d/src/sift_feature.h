// Define sift feature structure.
#ifndef __SIFT_FEATURE_H__
#define __SIFT_FEATURE_H__
#include <fstream>
#include <vector>
namespace SIFT {
    using std::vector;
    using std::istream;
    using std::ostream;

    // Define feature point structure and operator.
    struct FeaturePoint {
	
	static const int NUM_OF_DIMENSION = 128;
	FeaturePoint() {}
	FeaturePoint( const double x, const double y ) : x(x), y(y) {}

	double x, y, scale, orientation;
	unsigned char desc[NUM_OF_DIMENSION];
    };
    
    bool operator ==( const FeaturePoint &p1, const FeaturePoint &p2 ) {
	return p1.x == p2.x && p1.y == p2.y; 
    }

    istream &operator >>( istream &stream, FeaturePoint &p ) {

	stream >> p.y >> p.x >> p.scale >> p.orientation;
	for (unsigned int i = 0; i < FeaturePoint::NUM_OF_DIMENSION; ++i ) {
	    unsigned int v;
	    stream >> v;
	    p.desc[i] = v;
	}
	return stream;
    }; 

    ostream &operator <<( ostream &stream, const FeaturePoint &p ) { // notice the const!

	stream << p.y << " " << p.x << " " << p.scale << " " << p.orientation << " ";
	for (unsigned int i = 0; i < FeaturePoint::NUM_OF_DIMENSION; ++i ) {
	    stream << (unsigned int)(p.desc[i]) << " ";
	} 
	return stream;
    };

    // Define feature list by inherit vector class.
    struct FeatureList : public vector<FeaturePoint> {

	bool exist( const double x, const double y ) {
	    for (unsigned int i = 0; i < this->size(); ++i) {
		if ( (*this)[i].x == x && (*this)[i].y == y )
		    return true;
	    }
	    return false;
	}

	bool load( const char *filename ) {
	    std::ifstream in(filename);
	    if (!in.is_open()) 
		return true; // file_is_not_exist

	    int num_of_features;
	    int num_of_dimension;
	    resize(0); // vector initalization

	    in >> num_of_features >> num_of_dimension;
	    for (unsigned int i = 0; i < num_of_features; ++i) {
		FeaturePoint p;
		in >> p;
		// Check if p is a redundance feature via == operator was defined above.
		if (std::find(begin(), end(), p) == end())
		    push_back(p);
	    }
	    in.close();
	    return false;
	}
	
	// Loading without checking duplicated
	bool load_raw( const char *filename ) {
	    std::ifstream in(filename);
	    if (!in.is_open()) 
		return true; // file_is_not_exist

	    int num_of_features;
	    int num_of_dimension;
	    resize(0); // vector initalization

	    in >> num_of_features >> num_of_dimension;
	    for (unsigned int i = 0; i < num_of_features; ++i) {
		FeaturePoint p;
		in >> p;
		push_back(p);
	    }
	    in.close();
	    return false;
	}

    };

    // Load %04d.key file in a given dir
    class FeatureListLoader {
	public:
	    FeatureListLoader( const char* key_dir, const char* image_name, const char image_postfix ) : _dir(key_dir), _image_name(image_name), _image_postfix(image_postfix) {}

	    bool load( FeatureList &fl, const int frame_num ) const {
		char filename[128];
		sprintf( filename, "%s%s_%04d_%c.key", _dir.c_str(), _image_name, frame_num, _image_postfix );
		return fl.load( filename );
	    }
	    bool load_raw( FeatureList &fl, const int frame_num ) const {
		char filename[128];
		sprintf( filename, "%s%s_%04d_%c.key", _dir.c_str(), _image_name, frame_num, _image_postfix );
		return fl.load_raw( filename );
	    }
	private:
	    std::string _dir;
	    const char *_image_name;
	    const char _image_postfix;
    };
};
#endif
