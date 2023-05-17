#include<opencv2/highgui.hpp>
#include<opencv2/imgproc.hpp>
#include<math.h>
#include<iostream>
#include<string>
#include<sstream>
#include<iomanip>
#include<fstream>
#include"lowPrecise.h"
#include"highPrecise.h"

using namespace std;
using namespace cv;

const string inputpath = "ԭͼ\\";//�����ж���ͼƬ�����ļ���
const string outputpath = "���ͼ\\";//����ļ���



int main() {

	string filename;
	ifstream lowfile;
	ifstream highfile;
	vector<string>ALLFILENAMES;//�����ļ�������
	vector<string>INPUTPATHS;//�����ļ�������·��
	vector<string>OUTPUTPATHS;//�����ļ������·��
	string lowfileopenPath = inputpath + "lowfilenames.txt";
	string highfileopenPath = inputpath + "highfilenames.txt";

	lowfile.open(lowfileopenPath,ios::in);
	highfile.open(highfileopenPath,ios::in);

	if (!lowfile||!highfile) {
		cerr << "�޷����ļ���"<<endl;
		exit(0);
	}
	
	int count = 0;
	while (count<25) {
		getline(lowfile, filename, ',');
		ALLFILENAMES.push_back(filename); 
		count++;
	}
	while (count<29) {
		getline(highfile, filename, ',');
		ALLFILENAMES.push_back(filename); 
		count++;
	}
	
	for (int i=0;i<29;++i) {
		string WindowName = ALLFILENAMES[i];
		string inpath = inputpath + WindowName;
		string outpath = outputpath + WindowName;
		cv::Mat img = imread(inpath);
		cv::namedWindow(WindowName,WINDOW_NORMAL);
		if (i <= 24) {
			lowPreciseDetector ld(img);
			ld.showResult();
		}
		else if(i<=26){
			highPreciseDetector hd(img);
			hd.showScale1Result();
		}
		else {
			highPreciseDetector hd(img);
			hd.showScale2Result();
		}
		cv::resizeWindow(WindowName, Size(400, 300));
		imshow(WindowName, img); cout << setw(20) << setiosflags(ios_base::left) << WindowName << "    �Ѽ���";
		imwrite(outpath, img); cout <<setw(12)<< setiosflags(ios_base::left) <<" " << " ��д��:   " << outpath << endl;
	}
	std::cout << endl;
	std::cout << endl;
	std::cout << "����q�����˳�..." << endl;
	string c;

	int x=waitKey(0);
	
	if ((char)x == 'q') {
		destroyAllWindows();
		exit(0);
	}
	return 0;

}

