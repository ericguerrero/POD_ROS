#include "podescriber.h"

void PODescriber::_load_gabor_filter(string configPath){
    string fileName=configPath+_POD_GABOR_FNAME_;
    FILE *fileID=fopen(fileName.c_str(),"rt");
    float curValue;
    for (int iScale=0;iScale<_POD_GABOR_SCALES_;iScale++){
        for (int iOrient=0;iOrient<_POD_GABOR_ORIENT_;iOrient++){
            _gaborFilterBank[iScale][iOrient]=Mat::zeros(_POD_GABOR_ROWS_,_POD_GABOR_COLS_,CV_32FC1);
            for (int gRow=0;gRow<_POD_GABOR_ROWS_;gRow++){
                for (int gCol=0;gCol<_POD_GABOR_COLS_;gCol++){
                    fscanf(fileID,"%f",&curValue);
                    fscanf(fileID,",");
                    _gaborFilterBank[iScale][iOrient].at<float>(gRow,gCol)=curValue;
                }
            }
        }
    }
}

inline void PODescriber::_describe_patch(int patchChan, int patchRow, int patchCol){
    Mat convolvedPatch(_POD_PATCH_ROWS_,_POD_GABOR_COLS_,CV_32FC1);
    Mat thePatch(_theChannels[patchChan],Rect(patchCol*_POD_PATCH_COLS_,patchRow*_POD_PATCH_ROWS_,_POD_PATCH_COLS_,_POD_PATCH_ROWS_));
    float localEnergy,meanAmplitude,*p;
    for (int curScale=0;curScale<_POD_GABOR_SCALES_;curScale++){
        for (int curOrient=0;curOrient<_POD_GABOR_ORIENT_;curOrient++){
            localEnergy=0;
            meanAmplitude=0;
            filter2D(thePatch,convolvedPatch,CV_32FC1,_gaborFilterBank[curScale][curOrient]);
            assert(convolvedPatch.isContinuous());
            p=convolvedPatch.ptr<float>(0);
            for (int i=0;i<convolvedPatch.rows*convolvedPatch.cols;i++){
                localEnergy+=p[i]*p[i];
                meanAmplitude+=(p[i]>0?p[i]:-p[i]);
            }
            _imageDescriptor[patchRow][patchCol][0][patchChan][curScale][curOrient]=(float)localEnergy;
            _imageDescriptor[patchRow][patchCol][1][patchChan][curScale][curOrient]=(float)meanAmplitude;
        }
    }
}

void PODescriber::_thread_describe_image(int pRow){
    for (int pCol=0;pCol<_POD_NPATCH_COL_;pCol++){
        for (int nChan=0;nChan<_POD_IMAGE_NCHAN_;nChan++){
            _describe_patch(nChan,pRow,pCol);
        }
    }
}

void PODescriber::_prepare_input_images() {
    // Assert image is loaded
    assert(_originalImage.data);
    // Prepare the working image
    _sourceImage=_originalImage.clone();
    #if _POD_IMAGE_NCHAN_==1
    cvtColor(_sourceImage, _sourceImage, CV_BGR2GRAY);
    #endif
    // Assert number of channels
    assert(_sourceImage.channels()==_POD_IMAGE_NCHAN_);
    // Resize to desired width and height
    resize(_sourceImage,_sourceImage,Size(_POD_IMAGE_COLS_,_POD_IMAGE_ROWS_));
}

PODescriber::PODescriber(string configPath){
    _load_gabor_filter(configPath);
}

void PODescriber::load_image(string fileName){
    // Load the image as-is for drawing purposes.
    _originalImage=imread(fileName, CV_LOAD_IMAGE_COLOR);
    // Prepare them to be used
    _prepare_input_images();
}

void PODescriber::describe_image(){
    // Get the channels
    split(_sourceImage,_theChannels);

    #ifdef _POD_THREADED_
    // Threaded version
    thread t[_POD_NPATCH_ROW_];
    // Split channels in patches and describe them in sepparate threads
    for (int pRow=0;pRow<_POD_NPATCH_ROW_;pRow++){
        t[pRow]=thread(&PODescriber::_thread_describe_image,this,pRow);
    }
    // Join the threads with the main thread
    for (int pRow=0;pRow<_POD_NPATCH_ROW_;pRow++){
        t[pRow].join();
    }
    #else
    // Unthreaded version
    for (int pRow=0;pRow<_POD_NPATCH_ROW_;pRow++){
        _thread_describe_image(pRow);
    }
    #endif
}

void PODescriber::set_image(Mat &theImage) {
    _originalImage=theImage.clone();
    // Prepare them to be used
    _prepare_input_images();
}

Mat PODescriber::get_internal_image(){
    return _sourceImage;
}

Mat PODescriber::get_image() {
    return _originalImage;
}

float *PODescriber::get_descriptor(){
    return (float *)_imageDescriptor;
}