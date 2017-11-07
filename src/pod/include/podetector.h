// Detects PO at a patch level using a pre-trained SVM.
// Usage example:
// PODetector theDetector("TRAINED_SVM.XML");
// theDetector.load_image("00002.png");
// theDetector.classify_image();
// Mat c=theDetector.get_classification();
// Author   : A. Burguera.
// Creation : 21-Jan-2017

#ifndef _PODETECTOR_H_
#define _PODETECTOR_H_

#include "podefines.h"
#include "podescriber.h"
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

class PODetector {
private:
    Ptr<ml::SVM> _theSVM;           // The trained SVM
    PODescriber _theDescriber;      // Describer for the working image
    Mat _theClassification;         // The classification result
    float *_pDescriptor;         // The descriptor itself. First two indexes allow selecting at patch level.
    Mat _theImage;                  // Copy of the image being classified, just for drawing purposes.
    // Prepares the classification to be plotted as an overlay onto the image. Just for drawing.
    Mat _get_classification_overlay(float weightRed=1, float weightGreen=0, float weightBlue=0);

public:
    // Creates and loads the SVM from file (XML-OpenCV3 format)
    PODetector(string configPath);
    // Loads an image from file and prepares the describer.
    void load_image(string fileName);
    // Same as before, but with a pre-loaded image
    void set_image(Mat theImage);
    // Classifies the image at patch-level
    void classify_image();
    // Get descriptor ffrom podescriber
    float *get_descriptor();
    // Returns the original image.
    Mat get_image();
    // Gets the classification as a matrix of labels.
    Mat get_classification();
    // Gets the classification as an image ready to plot.
    Mat get_graphic_classification(int smoothContour=1, int doOverlay=1, float weightRed=0.5, float weightGreen=0, float weightBlue=0);
    // Gets the classification as a binary image ready to plot.
    Mat get_binary_classification();
    // Gets the mask
    Mat get_mask(int toImageSize=0);
};

#endif