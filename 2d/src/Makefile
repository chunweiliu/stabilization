CXXFLAGS=`pkg-config --cflags opencv gsl` -O3
LDFLAGS=`pkg-config --libs opencv gsl`

all: image_sift sift_fund_match sift_tracking kez_stabilize_estimate stabilize_estimate merge_path image_warp
	mv -iv image_sift ../bin/
	mv -iv sift_fund_match ../bin/
	mv -iv sift_tracking ../bin/
	mv -iv kez_stabilize_estimate ../bin/
	mv -iv stabilize_estimate ../bin/
	mv -iv merge_path ../bin/
	mv -iv image_warp ../bin/

sift_tracking: sift_tracking.cpp ../lib/libANN.a sift_feature.h
	${CXX} ${CXXFLAGS} -o sift_tracking sift_tracking.cpp ../lib/libANN.a ${LDFLAGS}

surf_tracking: surf_tracking.cpp ../lib/libANN.a
	${CXX} ${CXXFLAGS} -o surf_tracking surf_tracking.cpp ../lib/libANN.a ${LDFLAGS}

sift_match: sift_match.cpp ../lib/libANN.a sift_feature.h
	${CXX} ${CXXFLAGS} -o sift_match sift_match.cpp ../lib/libANN.a ${LDFLAGS}
	
sift_fund_match: sift_fund_match.cpp ../lib/libANN.a sift_feature.h
	${CXX} ${CXXFLAGS} -o sift_fund_match sift_fund_match.cpp ../lib/libANN.a ${LDFLAGS}
