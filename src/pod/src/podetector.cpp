#include "podetector.h"

Mat PODetector::_get_classification_overlay(float weightRed, float weightGreen, float weightBlue) {
    Mat matClassification=get_classification().clone();
    Mat theChannels[3];
    matClassification=1-matClassification;
    cvtColor(matClassification,matClassification,CV_GRAY2RGB);
    split(matClassification,theChannels);
    theChannels[0]=weightBlue*theChannels[0];
    theChannels[1]=weightGreen*theChannels[1];
    theChannels[2]=weightRed*theChannels[2];
    merge(theChannels,3,matClassification);
    return matClassification;
}


PODetector::PODetector(string configPath):_theDescriber(configPath) {
    _theSVM=ml::SVM::create();
    _theSVM=ml::SVM::load(configPath+_POD_SVM_FNAME_);
}

void PODetector::load_image(string fileName) {
    _theDescriber.load_image(fileName);
    _theImage=_theDescriber.get_image().clone();
    assert((_theImage.type()&7)==0);
}

void PODetector::set_image(Mat theImage) {
    _theDescriber.set_image(theImage);
    _theImage=_theDescriber.get_image().clone();
    assert((_theImage.type()&7)==0);
}

void PODetector::classify_image() {
    _theDescriber.describe_image();
    _pDescriptor = _theDescriber.get_descriptor();
    Mat descriptorMat(_POD_NPATCH_ROW_*_POD_NPATCH_COL_, _POD_DESCR_TYPES_*_POD_IMAGE_NCHAN_*_POD_GABOR_SCALES_*_POD_GABOR_ORIENT_, CV_32FC1, _pDescriptor);
    _theSVM->predict(descriptorMat, _theClassification, false);
}

Mat PODetector::get_image() {
    return _theImage;
}

float *PODetector::get_descriptor() {
    return _pDescriptor;
}

Mat PODetector::get_classification() {
    return _theClassification.reshape(1,_POD_NPATCH_ROW_);
}

Mat PODetector::get_graphic_classification(int smoothContour, int doOverlay, float weightRed, float weightGreen, float weightBlue) {
    Mat theImage=_theImage.clone();
    Mat classImage;
    int rowsOrig=theImage.rows;
    int colsOrig=theImage.cols;
    if (theImage.channels()==1) {
        cvtColor(theImage,theImage,CV_GRAY2RGB);
    }
    resize(_get_classification_overlay(weightRed,weightGreen,weightBlue),classImage,Size(colsOrig,rowsOrig),0,0,smoothContour?INTER_CUBIC:INTER_NEAREST);
    classImage.convertTo(classImage,CV_8UC3,255,0);
    if (doOverlay) {
        addWeighted(theImage, 0.5, classImage, 0.5, 0.0, classImage);
    }
    return classImage;
}

Mat PODetector::get_binary_classification() {
    Mat theImage=_theImage.clone();
    Mat classImage;
    int rowsOrig=theImage.rows;
    int colsOrig=theImage.cols;
    resize(get_classification(),classImage,Size(colsOrig,rowsOrig),0,0,INTER_NEAREST);
    classImage.convertTo(classImage,CV_8UC1,255,0);
    return classImage;
}

Mat PODetector::get_mask(int toImageSize) {
    Mat outImage;
    get_classification().convertTo(outImage, CV_8UC1, 255.0);
    if (toImageSize) {
        resize(outImage,outImage,Size(_theImage.cols,_theImage.rows),0,0,INTER_NEAREST);
    }
    return outImage;
}


    