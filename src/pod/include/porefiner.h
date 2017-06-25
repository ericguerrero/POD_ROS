// Refines a detection mask by changing the class of the
// contour pixels of each class if their color is closer
// to the mean color of the other class.
// Usage example:
// PORefiner theRefiner();
// theRefiner.set_data(theImage,theMask);
// theRefiner.process();
// Mat newMask=theRefiner.get_refined().clone();
// Author   : A. Burguera.
// Based on code by J.L. Lisani
// Creation : 24-Jan-2017

#ifndef _POREFINER_H_
#define _POREFINER_H_

#include "podefines.h"
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

// Some macros to speed-up matrix accesses
#define _POR_GET_PATCH_(patchRow, patchCol) (_theMask.at<uchar>(patchRow, patchCol))
#define _POR_GET_PIXEL_(nChan, rChan, cChan) (_theChannels[nChan].at<uchar>(rChan,cChan))
#define _POR_GET_OMVAL_(rMask, cMask) (_outMask.at<uchar>(rMask,cMask))
#define _POR_SET_OMVAL_(rMask, cMask, vMask) {_outMask.at<uchar>(rMask,cMask)=vMask;}
#define _POR_PATCH_EXISTS_(patchRow, patchCol) (patchRow>=0 && patchRow<_POD_NPATCH_ROW_ && patchCol>=0 && patchCol<_POD_NPATCH_COL_)
#define _POR_PIXEL_EXISTS_(rChan,cChan) (rChan>=0 && rChan<_chanRows && cChan>=0 && cChan<_chanCols)
#define _POR_PIXEL_TO_PATCH_(rPix, cPix, rPatch, cPatch) {rPatch=rPix*_POD_NPATCH_ROW_/_chanRows;cPatch=cPix*_POD_NPATCH_COL_/_chanCols;}

class PORefiner {
private:
    Mat _originalImage;     // The image provided to set_data
    Mat _theMask;           // The label matrix to be refined
    Mat *_theChannels;      // Original image channels
    int _numChannels;       // Number of channels (ideally 3)
    int _chanRows;          // Original image height
    int _chanCols;          // Original image width
    int *_meanColorsPO;     // Mean color of internal PO patches
    int *_meanColorsNoPO;   // Mean color of internal non-PO patches
    int *_curMeanColor;     // Mean color of the neighborhood under analysis
    Mat _outMask;           // The refined mask
    int _nothingToProcess;  // Set if no refinement can be done

    // Releases allocated memory if any
    void _release_data();
    // Computes the mean color of the neighborhood of the specified pixel
    void _compute_cur_mean_color(int imRow, int imCol);
    // Manhattan distance between two colors
    int _color_distance(int *c1, int *c2);
    // Tells if a patch is 4-connected to patches of the same value pVal
    int _is_connected(int pRow, int pCol, uchar pVal);
    // Compute mean colors of internal patches of each class.
    // An internal patch is 4-connected to patches of the same type.
    int _compute_color_means();
    // Tells if an pixel is at the edge of a class
    int _is_border(int omRow, int omCol, uchar theValue);
    // Refine the specified label
    void _refine(uchar theValue);

public:
    // Initializes pointers.
    PORefiner();
    // Destructor. Just release data.
    ~PORefiner();
    // Release the allocated data.
    void set_data(Mat inputImage, Mat theMask);
    // Do the refinement.
    void process();
    // Provide the refined mask with the working size.
    Mat get_refined();
    // Provide the refined mask scaled to the image size
    Mat get_refined_scaled();
    // Same as before, but also can overlay over the original image
    Mat get_refined_overlayed(int smoothContour=1, int doOverlay=1, float weightRed=0, float weightGreen=0.5, float weightBlue=0);
};

#endif