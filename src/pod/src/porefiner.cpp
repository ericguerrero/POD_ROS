#include "porefiner.h"

void PORefiner::_release_data() {
    if (_meanColorsPO!=NULL) delete _meanColorsPO;
    if (_meanColorsNoPO!=NULL) delete _meanColorsNoPO;
    if (_curMeanColor!=NULL) delete _curMeanColor;
}

void PORefiner::_compute_cur_mean_color(int imRow, int imCol) {
    int rowMin=(imRow-_POD_COLOR_RADIUS_)>0?(imRow-_POD_COLOR_RADIUS_):0;
    int rowMax=(imRow+_POD_COLOR_RADIUS_)<_chanRows?(imRow+_POD_COLOR_RADIUS_):_chanRows-1;
    int colMin=(imCol-_POD_COLOR_RADIUS_)>0?(imCol-_POD_COLOR_RADIUS_):0;
    int colMax=(imCol+_POD_COLOR_RADIUS_)<_chanCols?(imCol+_POD_COLOR_RADIUS_):_chanCols-1;
    int curChan, curRow, curCol, cCount=0;

    for (curChan=0;curChan<_numChannels;curChan++) {
        _curMeanColor[curChan]=0;
        cCount=0;
        for (curRow=rowMin;curRow<=rowMax;curRow++) {
            for (curCol=colMin;curCol<=colMax;curCol++) {
                _curMeanColor[curChan]+=(int)(_POR_GET_PIXEL_(curChan,curRow,curCol));
                cCount++;
            }
        }
        _curMeanColor[curChan]/=cCount;
    }
}

int PORefiner::_color_distance(int *c1, int *c2) {
    float theDistance=0;
    for (int i=0;i<_numChannels;i++) {
        theDistance+=abs(c1[i]-c2[i]);
    }
    return theDistance;
}

int PORefiner::_is_connected(int pRow, int pCol, uchar pVal) {
    if (_POR_PATCH_EXISTS_(pRow-1,pCol) && _POR_GET_PATCH_(pRow-1,pCol)!=pVal) return 0;
    if (_POR_PATCH_EXISTS_(pRow,pCol+1) && _POR_GET_PATCH_(pRow,pCol+1)!=pVal) return 0;
    if (_POR_PATCH_EXISTS_(pRow+1,pCol) && _POR_GET_PATCH_(pRow+1,pCol)!=pVal) return 0;
    if (_POR_PATCH_EXISTS_(pRow,pCol-1) && _POR_GET_PATCH_(pRow,pCol-1)!=pVal) return 0;
    return 1;
}

int PORefiner::_compute_color_means() {
    int nChan, rChan, cChan, rPatch, cPatch, patchConnected;
    uchar vPatch;
    int numPO, numNOPO;
    for (nChan=0;nChan<_numChannels;nChan++) {
        numPO=0;
        numNOPO=0;
        _meanColorsPO[nChan]=0;
        _meanColorsNoPO[nChan]=0;
        for (rChan=0;rChan<_chanRows;rChan++) {
            for (cChan=0;cChan<_chanCols;cChan++) {
                _POR_PIXEL_TO_PATCH_(rChan,cChan,rPatch,cPatch);
                vPatch=_POR_GET_PATCH_(rPatch,cPatch);
                patchConnected=_is_connected(rPatch,cPatch,vPatch);
                if (patchConnected) {
                    switch (vPatch) {
                        case _POD_LABEL_PO_:
                        numPO++;
                        _meanColorsPO[nChan]+=(int)(_POR_GET_PIXEL_(nChan,rChan,cChan));
                        break;
                        case _POD_LABEL_NOPO_:
                        numNOPO++;
                        _meanColorsNoPO[nChan]+=(int)(_POR_GET_PIXEL_(nChan,rChan,cChan));
                        break;
                    }
                }
            }
        }
        if (!numPO || !numNOPO) return -1;
        _meanColorsPO[nChan]/=numPO;
        _meanColorsNoPO[nChan]/=numNOPO;
    }
    return 0;
}

int PORefiner::_is_border(int omRow, int omCol, uchar theValue) {
    if ((_POR_PIXEL_EXISTS_(omRow-1,omCol)) && (_POR_GET_OMVAL_(omRow-1,omCol)!=theValue)) return 1;
    if ((_POR_PIXEL_EXISTS_(omRow,omCol+1)) && (_POR_GET_OMVAL_(omRow,omCol+1)!=theValue)) return 1;
    if ((_POR_PIXEL_EXISTS_(omRow+1,omCol)) && (_POR_GET_OMVAL_(omRow+1,omCol)!=theValue)) return 1;
    if ((_POR_PIXEL_EXISTS_(omRow,omCol-1)) && (_POR_GET_OMVAL_(omRow,omCol-1)!=theValue)) return 1;
    return 0;
}

void PORefiner::_refine(uchar theValue) {
    if (_nothingToProcess) return;
    int omRow, omCol, dPO, dNoPO, nChanges;
    do {
        nChanges=0;
        for (omRow=0;omRow<_chanRows;omRow++) {
            for (omCol=0;omCol<_chanCols;omCol++) {
                if (_POR_GET_OMVAL_(omRow,omCol)==theValue) {
                    if (_is_border(omRow,omCol,theValue)) {
                        _compute_cur_mean_color(omRow,omCol);
                        dPO=_color_distance(_curMeanColor,_meanColorsPO);
                        dNoPO=_color_distance(_curMeanColor,_meanColorsNoPO);
                        if (theValue==_POD_LABEL_PO_) {
                            if (dPO>dNoPO) {
                                _POR_SET_OMVAL_(omRow,omCol,_POD_LABEL_NOPO_);
                                nChanges++;
                            }
                        } else if (theValue==_POD_LABEL_NOPO_){
                            if (dPO<dNoPO) {
                                _POR_SET_OMVAL_(omRow,omCol,_POD_LABEL_PO_);
                                nChanges++;
                            }
                        }
                    }
                }
            }
        }
    } while (nChanges);
}

PORefiner::PORefiner() {
    _meanColorsPO=NULL;
    _meanColorsNoPO=NULL;
    _curMeanColor=NULL;
}


PORefiner::~PORefiner() {
    _release_data();
}

void PORefiner::set_data(Mat inputImage, Mat theMask) {
    _originalImage=inputImage;
    Mat theImage;
    resize(inputImage, theImage,Size(_POD_REFINE_COLS_,_POD_REFINE_ROWS_),0,0,INTER_CUBIC);
    // Assert the image is CV_8U
    assert((theImage.type()&7)==0);
    // Store image size
    _chanRows=theImage.rows;
    _chanCols=theImage.cols;
    // Prepare the mask
    theMask.convertTo(_theMask, CV_8UC1);
    // Assert the mask is CV_8UC1
    assert(_theMask.type()==CV_8UC1);
    // Assert the mask is at patch level
    assert(_theMask.rows==_POD_NPATCH_ROW_ && _theMask.cols==_POD_NPATCH_COL_);
    // Release pointers if the object is re-used.
    _release_data();
    // Allocate space for the channels
    _numChannels=theImage.channels();
    _theChannels=new Mat[_numChannels];
    // Get the channels
    split(theImage,_theChannels);
    // Get space for mean colors
    _meanColorsPO=new int[_numChannels];
    _meanColorsNoPO=new int[_numChannels];
    _curMeanColor=new int[_numChannels];
    // Compute them
    _nothingToProcess=_compute_color_means();
    // Prepare outMask
    resize(_theMask,_outMask,Size(_chanCols,_chanRows),0,0,INTER_NEAREST);
}

void PORefiner::process() {
    _refine(_POD_LABEL_PO_);
    _refine(_POD_LABEL_NOPO_);
}

Mat PORefiner::get_refined() {
    Mat outMask;
    cvtColor(_outMask,outMask,CV_GRAY2RGB);
    outMask.convertTo(outMask,CV_8UC3,255,0);
    return outMask;
}

Mat PORefiner::get_refined_scaled() {
    Mat scaledMask;
    resize(_outMask*255,scaledMask,Size(_originalImage.cols,_originalImage.rows),0,0,INTER_CUBIC);
    cvtColor(scaledMask,scaledMask,CV_GRAY2RGB);
    return scaledMask;
}

Mat PORefiner::get_refined_overlayed(int smoothContour, int doOverlay, float weightRed, float weightGreen, float weightBlue) {
    Mat scaledMask=_outMask.clone();
    Mat theChannels[3];
    scaledMask=1-scaledMask;
    scaledMask.convertTo(scaledMask,CV_32FC1);
    cvtColor(scaledMask,scaledMask,CV_GRAY2RGB);

    split(scaledMask,theChannels);
    theChannels[0]=weightBlue*theChannels[0];
    theChannels[1]=weightGreen*theChannels[1];
    theChannels[2]=weightRed*theChannels[2];
    merge(theChannels,3,scaledMask);

    Mat theImage=_originalImage.clone();
    int rowsOrig=theImage.rows;
    int colsOrig=theImage.cols;
    if (theImage.channels()==1) {
        cvtColor(theImage,theImage,CV_GRAY2RGB);
    }

    resize(scaledMask,scaledMask,Size(colsOrig,rowsOrig),0,0,smoothContour?INTER_CUBIC:INTER_NEAREST);

    scaledMask.convertTo(scaledMask,CV_8UC3,255,0);
    if (doOverlay) {
        addWeighted(theImage, 0.5, scaledMask, 0.5, 0.0, scaledMask);
    }
    return scaledMask;
}
