//
//  Calibrator.m
//  ios7tester
//
//  Created by Sam Royston on 5/9/15.
//  Copyright (c) 2015 Sam Fox. All rights reserved.
//

#import "Calibrator.h"

@implementation Calibrator

int calibrateUntil = 50;
cv::Mat cameraSave;
cv::Mat distortionSave;
double lowErr = 100000000000;


- (id)init{
    if (self == [super init]) {
        
        
    }
    return self;
}

std::vector<cv::Point3f> Create3DChessboardCorners(cv::Size boardSize, float squareSize)
{
    // This function creates the 3D points of your chessboard in its own coordinate system
    
    std::vector<cv::Point3f> corners;
    
    for( int i = 0; i < boardSize.height; i++ )
    {
        for( int j = 0; j < boardSize.width; j++ )
        {
            corners.push_back(cv::Point3f(float(j*squareSize),
                                          float(i*squareSize), 0));
        }
    }
    
    return corners;
}

- (void) processImage:(cv::Mat &)image{
    cv::Size patternsize(7,6
                         
                         );
    cv::Mat gray;
    cv::cvtColor(image,  gray, CV_BGR2GRAY); //source image
    std::vector<std::vector<cv::Point2f> > corners(1);
 //this will be filled by the detected corners
    
    //CALIB_CB_FAST_CHECK saves a lot of time on images
    //that do not contain any chessboard corners
    std::vector<cv::Mat> rotationVectors;
    std::vector<cv::Mat> translationVectors;
    bool patternfound = findChessboardCorners(gray, patternsize, corners[0],
                                              cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE
                                              + cv::CALIB_CB_FAST_CHECK);
    cv::Mat distortionCoefficients = cv::Mat::zeros(8, 1, CV_64F); // There are 8 distortion coefficients
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    if (patternfound ){
        //cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
        //cvTermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
        drawChessboardCorners(image, patternsize, cv::Mat(corners[0]), TRUE);
        std::vector<std::vector<cv::Point3f> > objectPoints(1);
        objectPoints[0] = Create3DChessboardCorners(patternsize, 1.0f);
        double err = cv::calibrateCamera(objectPoints, corners, patternsize, cameraMatrix, distortionCoefficients, rotationVectors, translationVectors);
        if (err < lowErr) {
            lowErr = err;
            cameraMatrix.copyTo(cameraSave);
            distortionCoefficients.copyTo(distortionSave);
            [Calibrator storeCVMat:cameraSave withKey:@"cameraMatrix"];
            [Calibrator storeCVMat:distortionSave withKey:@"distortionMatrix"];
            
            NSLog(@"camera matrix saved");
        }
        std::cout << "Error: " << lowErr <<std::endl;
        calibrateUntil++;
    }
    
    std::cout  << std::endl;
}

+(void) storeCVMat: (cv::Mat) mat withKey: (NSString*) key{
    
    int matRows = mat.rows;
    int matCols = mat.cols;
    NSMutableArray *matArray = [NSMutableArray arrayWithCapacity: matRows * matCols];
    NSNumber* matElemnt;
    for (int i=0 ; i< matRows * matCols; i++)
    {
        if (matRows != 1 && matCols != 1) {
            matElemnt = [NSNumber numberWithDouble:mat.at<double>(i/matCols,i%matCols)];
        }
        else {
            matElemnt = [NSNumber numberWithFloat:mat.at<double>(i)];
        }
        [matArray insertObject:matElemnt atIndex:i];
    }
    [[NSUserDefaults standardUserDefaults] setObject:@{@"mat":matArray, @"rows":[NSNumber numberWithInt:matRows], @"cols":[NSNumber numberWithInt:matCols]} forKey:key];
    [[NSUserDefaults standardUserDefaults] synchronize];
}

+(cv::Mat) loadMatFromKey: (NSString*) key{
    
    try {
        NSDictionary* matObject = [[NSUserDefaults standardUserDefaults] objectForKey:key];
        int matRows = [matObject[@"rows"] integerValue];
        int matCols = [matObject[@"cols"] integerValue];
        NSArray *matArray = matObject[@"mat"];
        cv::Mat output = cv::Mat::zeros(matRows, matCols, CV_64F);
        
        
        for(NSNumber *element in matArray){
            int index = [matArray indexOfObject:element];
            int row = index / matCols;
            int col = index % matCols;
            output.at<double>(row,col) = [element doubleValue];
        }
    return output;
    } catch (errno_t) {
        NSLog(@"Matrix not found for key '%@'", key);
    }
    
    return cv::Mat::zeros(1, 1,CV_64F);
    
}

@end



