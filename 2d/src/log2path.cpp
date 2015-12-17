#include <iostream>
#include <fstream>
#include <cstdio>
#include <cmath>
using namespace std;

int main( int argc, char *argv[] )
{
    if (argc != 3) {
	cerr << "Usage: " << argv[0] 
	     << " <deshaker_log> <path_file>"
	     << endl;
	return -1;
    }

    const char *deshaker_file	= argv[1];
    const char *output_pathfile = argv[2];

    char buf[128];    
    //FILE *fid_in = fopen( deshaker_file, 'r' );
    //FILE *fid_out = fopen( output_pathfile, 'r' );
    //while( fgets(buf, 128, fid_in) != NULL ) {
    ifstream in_deshaker(deshaker_file);
    ofstream out_file(output_pathfile);
    out_file << "1 0 0 0" << endl;
    while( !in_deshaker.eof() ) {

	int frame;
	float panx, pany, degree, scale;
	in_deshaker >> frame >> panx >> pany >> degree >> scale;

	//sscanf( buf, "%d%f%f%f%f", frame, panx, pany, degree, scale );

	const float radious = degree/180.;
	//fprintf( fid_out, "%f %f %f %f\n", scale, radious, panx, pany );
	out_file << scale << " " << radious << " " << panx << " " << pany << endl;
    }

    return 0;
}
