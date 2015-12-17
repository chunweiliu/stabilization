#include <iostream>
#include <fstream>
using namespace std;

int main ( int argc, char *argv[] )
{
    if (argc != 3) {
	cerr << "Usage: " << argv[0]
	     << " <input_path_1> <input_path_2>"
	     << endl;
    }

    const char *input_path_name1 = argv[1];
    const char *input_path_name2 = argv[2];

    ifstream in1;
    in1.open( input_path_name1 );
        
    ifstream in2;
    in2.open( input_path_name2 );

    if (!in1.is_open() || !in2.is_open()) {
	cerr << "Input file is not exist." << endl;
	return -1;
    }

    ofstream out;
    int frame_num = 0;
    while (1) {

	double a[4], b[4];
	/*
	for (unsigned int i = 0; i < 4; ++i) {
	    in1 >> a[i];
	    in2 >> b[i];
	    cout << (a[i] + b[i]) * 0.5 << " ";
	}
	*/
	//cout << endl;
	//cout << frame_num++ << endl;

	in1 >> a[0] >> a[1] >> a[2] >> a[3];
	if (in1.eof()) break;

	in2 >> b[0] >> b[1] >> b[2] >> b[3];
	cout << 0.5*(a[0]+b[0]) << " "
	     << 0.5*(a[1]+b[1]) << " "
	     << 0.5*(a[2]+b[2]) << " "
	     << 0.5*(a[3]+b[3]) << endl;
    } 
    in1.close();
    in2.close();
    
    
    return 0;
}
