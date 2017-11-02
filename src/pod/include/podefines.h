// Defines used by different PO classes.
// Author   : A. Burguera.
// Creation : 21-Jan-2017

#ifndef _PODEFINES_H_
#define _PODEFINES_H_

// Conditional compiling (comment/uncomment)
#define _POD_THREADED_              // Describe using (hoppefully parallel) threads
#define _POD_MAJORITY_DESC_         // If set, majority labels are used. Otherwise, all/none

// Global parameters
#define _POD_IMAGE_COLS_    320     // Images are resized to this width...
#define _POD_IMAGE_ROWS_    240     // ... and height.
#define _POD_IMAGE_NCHAN_   1       // Image channels. Set to 1 or 3
#define _POD_NPATCH_COL_    40      // Number of columns of patches.
#define _POD_NPATCH_ROW_    30      // Number of rows of patches.
#define _POD_GABOR_SCALES_  5       // Number of scales in the filter bank.
#define _POD_GABOR_ORIENT_  8       // Number of gabor orientations in f. bank.
#define _POD_GABOR_ROWS_    8       // Number of rows per filter.
#define _POD_GABOR_COLS_    8       // Number of columns per filter.
#define _POD_DESCR_TYPES_   2       // Used local energy and mean amplitude.
#define _POD_LABEL_PO_      0       // Label assigned to PO
#define _POD_LABEL_NOPO_    1       // Label assigned to PO
#define _POD_REFINE_COLS_   320     // Working width for refiner
#define _POD_REFINE_ROWS_   240     // Working height for refiner
#define _POD_COLOR_RADIUS_  3       // Radius for refiner to compute mean color
#define _POD_GABOR_FNAME_   "GABOR.CSV"      // CSV file with Gabor data.
#define _POD_SVM_FNAME_     "TRAINEDSVM.XML"    // XML file to save/load trained SVM.

// The following two must evaluate to integer
#define _POD_PATCH_ROWS_    (_POD_IMAGE_ROWS_/_POD_NPATCH_ROW_)
#define _POD_PATCH_COLS_    (_POD_IMAGE_COLS_/_POD_NPATCH_COL_)

#endif