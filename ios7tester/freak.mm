//
//  freak.m
//  ios7tester
//
//  Created by Sam Fox on 10/16/13.
//  Copyright (c) 2013 Sam Fox. All rights reserved.
//

#import "freak.h"

@implementation freak

cv::FREAK extractor;
cv::Mat img_object,homography;
cv::Mat descriptors_object;
std::vector<cv::KeyPoint> keypoints_object;
float nndrRatio = 0.7f;


- (id)init{
    if (self == [super init]) {
        
    
        #pragma mark TODO: get Mat from uiimage
        if(!self.templateImg){
            self.templateImg = [UIImage imageNamed:@"santita2.png"];
        }
        
        cv::Mat _temp = [self cvMatFromUIImage:self.templateImg];
        cv::cvtColor(_temp, img_object, CV_BGR2GRAY);
        cv::FAST(img_object, keypoints_object, 9);
        extractor.compute( img_object, keypoints_object, descriptors_object);
    }
    return self;
}

- (cv::Mat)cvMatFromUIImage:(UIImage *)image
{
    CGColorSpaceRef colorSpace = CGImageGetColorSpace(image.CGImage);
    CGFloat cols = image.size.width;
    CGFloat rows = image.size.height;

    cv::Mat cvMat(rows, cols, CV_8UC4); // 8 bits per component, 4 channels
    
    CGContextRef contextRef = CGBitmapContextCreate(cvMat.data,                 // Pointer to  data
                                                    cols,                       // Width of bitmap
                                                    rows,                       // Height of bitmap
                                                    8,                          // Bits per component
                                                    cvMat.step[0],              // Bytes per row
                                                    colorSpace,                 // Colorspace
                                                    kCGImageAlphaNoneSkipLast |
                                                    kCGBitmapByteOrderDefault); // Bitmap info flags
    
    CGContextDrawImage(contextRef, CGRectMake(0, 0, cols, rows), image.CGImage);
    CGContextRelease(contextRef);
    CGColorSpaceRelease(colorSpace);
    
    return cvMat;
}

- (void) processImage:(cv::Mat &)image{
    //cv::pyrDown(image, image);
    
    cv::Mat descriptors_scene;

    std::vector< cv::DMatch >   matches,good_matches;

    cv::Mat img_scene;
    cv::cvtColor(image, img_scene, CV_BGR2GRAY);
    std::vector<cv::KeyPoint> keypoints_scene;
    cv::FAST(img_scene, keypoints_scene, 9);
    
    

    //-- Step 2: Calculate descriptors (feature vectors)
    
    extractor.compute( img_scene, keypoints_scene, descriptors_scene );

    //-- Step 3: Matching descriptor vectors using binary Hamming matcher
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    
    matcher.match(descriptors_object, descriptors_scene, matches);
    
    good_matches = [self processMatches:matches];
    
    cv::vector<cv::Point2f> dscene,dobject;
    cv::vector<cv::Point2f> scene;
    cv::vector<cv::Point2f>obj;
    if (good_matches.size() > 4) {
        for( int i = 0; i < 20; i++ )
        {
            //-- Get the keypoints from the good matches
            obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
            scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
        }
        
        cv::Mat trans = cv::findFundamentalMat(obj,scene);

        
//        cv::Mat homo = cv::findHomography(obj, scene, CV_RANSAC);
//        if (homography.empty()) {
//            homo.copyTo(homography);
//        }
//        cv::accumulateWeighted(homo, homography, 0.1);

//        NSLog(@"rows: %i, columns: %i", homo.rows, homo.cols);
//        for( int j = 0; j < homo.rows; j++){
//            
//            for (int h = 0; h < homo.cols; h++) {
//                 NSLog(@"%f", homo.at<float>(j,h));
//            }
//        }
        
        std::vector<cv::Point2f> obj_corners(4);
        obj_corners[0] = cvPoint( img_object.cols / 2, img_object.rows / 2 );
        obj_corners[1] = cvPoint( img_object.cols / 2 - 2, img_object.rows / 2 - 2 );
        obj_corners[2] = cvPoint( img_object.cols / 2 - 2, img_object.rows / 2 + 2 );
        obj_corners[3] = cvPoint( 0, img_object.rows );
        std::vector<cv::Point2f> scene_corners(4);
        
        cv::perspectiveTransform( obj_corners, scene_corners, trans );
        
        cv::line( img_scene, obj_corners[0] , scene_corners[0] , cv::Scalar(255, 255, 255), 2 );
        cv::line( img_scene, scene_corners[0] , scene_corners[1] , cv::Scalar( 70, 0, 0), 5 );
        cv::line( img_scene, scene_corners[0] , scene_corners[2] , cv::Scalar( 70, 0, 0), 5 );
//      cv::line( img_scene, scene_corners[3] , scene_corners[0], cv::Scalar( 255, 255, 255), 4 );
    
    }
    
    cv::drawMatches( img_object, keypoints_object, img_scene, keypoints_scene, good_matches, image);
    
    keypoints_object = keypoints_scene;
    descriptors_object = descriptors_scene;
    img_object  = img_scene;

}

//- (void) drawMovement: (std::vector<<#class _Tp#>>)

- (std::vector< cv::DMatch >)processMatches:(std::vector< cv::DMatch >)matches{
    
    int max = 0;
    int min = 1000;
    std::vector< cv::DMatch > good_matches;
    for (int i = 0; i < matches.size(); i++ )
    {
        if(matches[i].distance < min)
        {
            min = matches[i].distance;
        }
        if(matches[i].distance > max)
        {
            max = matches[i].distance;
        }
    }
    for (int i = 0; i < matches.size(); i++ )
    {
        if(matches[i].distance < min + 0.3*(max-min))
        {
            good_matches.push_back(matches[i]);
        }
    }
    return good_matches;
}

@end
