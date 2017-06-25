// Class used to describe images in search of Posidonia.
// Usage example:
// PODescriber() theDescriber;
// theDescriber.load_image("TEST.png");
// theDescriber.describe_image();
// float *theDescribtor=theDescriber.get_descriptor();
// Author   : A. Burguera.
// Creation : 21-Jan-2017

#ifndef _PODESCRIBER_H_
#define _PODESCRIBER_H_

#include "podefines.h"
#ifdef _POD_THREADED_
#include <thread>
#endif
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class PODescriber {
private:
    // The image as it was provided to load_image
    Mat _originalImage;
    // The working image, adapted to the requirements (channels and size)
    Mat _sourceImage;
    // The channels. Just a copy of _sourceImage if only one channel is used.
    Mat _theChannels[_POD_IMAGE_NCHAN_];
    // The filter baks used to compute descriptors. Loaded from file.
    Mat _gaborFilterBank[_POD_GABOR_SCALES_][_POD_GABOR_ORIENT_];
    // The descriptor itself. First two indexes allow selecting at patch level.
    float _imageDescriptor[_POD_NPATCH_ROW_][_POD_NPATCH_COL_][_POD_DESCR_TYPES_][_POD_IMAGE_NCHAN_][_POD_GABOR_SCALES_][_POD_GABOR_ORIENT_];

    // Load Gabor filter data from file. The file is assumed to be correct.
    void _load_gabor_filter(string configPath);
    // Compute the descriptors for the specified patch
    inline void _describe_patch(int patchChan, int patchRow, int patchCol);
    // Intermediate image description used by describe_image
    void _thread_describe_image(int pRow);
    // Prepares the working image with the proper size and num. of channels.
    void _prepare_input_images();

public:
    // Constructor: Just load the Gabor filter bank
    PODescriber(string configPath);
    // Load image from file
    void load_image(string fileName);
    // Describes the image by computing a descriptor for each patch.
    void describe_image();
    // Same as load_image, for pre-loaded images
    void set_image(Mat &theImage);
    // Returns the working image. Must be there: no checks performed.
    Mat get_internal_image();
    // Returns the image originally provided to load_image. Must be there.
    Mat get_image();
    // Provides a pointer to the stored descriptor.
    float *get_descriptor();
};

#endif