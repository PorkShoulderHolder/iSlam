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
cv::Matx34d P0;
cv::Mat accT(0.0f,0.0f,0.0f);
cv::vector<cv::Point3d> pcl;
int thresh = 9;

struct Anchor {
    cv::Point3d pt;
    std::vector<int>index_of_2d_origin;
    cv::Point3d confidence;
};

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
        P0 = cv::Matx34d(1.0,    0.0, 0.0, 0.0,
                         0.0,    1.0, 0.0, 0.0,
                         0.0,    0.0, 1.0, 0.0);
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
    cv::FAST(img_scene, keypoints_scene, thresh);
    
  
    

    //-- Step 2: Calculate descriptors (feature vectors)
    
    extractor.compute( img_scene, keypoints_scene, descriptors_scene );

    //-- Step 3: Matching descriptor vectors using binary Hamming matcher
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    
    matcher.match(descriptors_object, descriptors_scene, matches);
    
    good_matches = matches;// [self processMatches:matches];
    
    if( matches.size() > 80 ){
        thresh++;
    }
    else if ( matches.size() < 80 ){
        thresh--;
    }
    
    
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
        
        cv::SVD svd(trans);
        
        cv::Matx33d W(0,-1,0,
                  1,0,0,
                  0,0,1);
        cv::Matx33d Winv(0,1,0,
                     -1,0,0,
                     0,0,1);
        cv::Mat R = svd.u * cv::Mat(W) * svd.vt;
        cv::Mat t = svd.u.col(2); //u3
        cv::Matx34d P1;
        cv::hconcat(R , t, P1);
        printf("--------\n");
        if( accT.size != t.size ){
            accT = t;
        }
        cv::accumulateWeighted(t, accT, 0.1);
        std::cout << std::endl <<        accT           << std::endl << std::endl;
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
        cv::vector<cv::Point3d> pointcloud;

        TriangulatePoints(keypoints_scene, keypoints_object, P0, P1, pointcloud);
        
        pcl = pointcloud;
        
    }
    
    cv::drawMatches( img_object, keypoints_object, img_scene, keypoints_scene, good_matches, image);
    
    keypoints_object = keypoints_scene;
    descriptors_object = descriptors_scene;
    img_object  = img_scene;
    
}

//- (void) drawMovement: (std::vector<<#class _Tp#>>)

cv::Mat_<double> LinearLSTriangulation(
                                   cv::Point3d u,//homogenous image point (u,v,1)
                                   cv::Matx34d P,//camera 1 matrix
                                   cv::Point3d u1,//homogenous image point in 2nd camera
                                   cv::Matx34d P1//camera 2 matrix
)
{
    //build A matrix
    cv::Matx43d A(u.x*P(2,0)-P(0,0),u.x*P(2,1)-P(0,1),u.x*P(2,2)-P(0,2),
              u.y*P(2,0)-P(1,0),u.y*P(2,1)-P(1,1),u.y*P(2,2)-P(1,2),
              u1.x*P1(2,0)-P1(0,0), u1.x*P1(2,1)-P1(0,1),u1.x*P1(2,2)-P1(0,2),
              u1.y*P1(2,0)-P1(1,0), u1.y*P1(2,1)-P1(1,1),u1.y*P1(2,2)-P1(1,2)
              );
    //build B vector
    cv::Matx41d B(-(u.x*P(2,3)-P(0,3)),
              -(u.y*P(2,3)-P(1,3)),
              -(u1.x*P1(2,3)-P1(0,3)),
              -(u1.y*P1(2,3)-P1(1,3)));
    //solve for X
    cv::Mat_<double> X;
    solve(A,B,X,cv::DECOMP_SVD);
    return X;
}

double TriangulatePoints(
                         const cv::vector<cv::KeyPoint>& pt_set1,
                         const cv::vector<cv::KeyPoint>& pt_set2,
                        // const cv::Mat&Kinv,
                         const cv::Matx34d& P,
                         const cv::Matx34d& P1,
                         cv::vector<cv::Point3d>& pointcloud)
{
    cv::vector<double> reproj_error;
    for (unsigned int i=0; i<fmin(pt_set1.size(), pt_set2.size()); i++) {
        //convert to normalized homogeneous coordinates
        cv::Point2f kp = pt_set1[i].pt;
        
        cv::Point3d u(kp.x,kp.y,1.0);
       // cv::Mat_<double> um = Kinv * cv::Mat_<double>(u);
      //  u = um.at<cv::Point3d>(0);
        cv::Point2f kp1 = pt_set2[i].pt;
        
        cv::Point3d u1(kp1.x,kp1.y,1.0);
        //cv::Mat_<double> um1 = Kinv * cv::Mat_<double>(u1);
        //u1 = um1.at<cv::Point3d>(0);
        
        //triangulate
        cv::Mat_<double> X = LinearLSTriangulation(u,P,u1,P1);
        
        //calculate reprojection error
        //cv::Mat_<double> xPt_img = K * cv::Mat(P1) * X;
        //cv::Point2f xPt_img_(xPt_img(0)/xPt_img(2),xPt_img(1)/xPt_img(2));
        //reproj_error.push_back(norm(xPt_img_-kp1));
        //store 3D point
        pointcloud.push_back(cv::Point3d(X(0),X(1),X(2)));
    }
    //return mean reprojection error
    cv::Scalar me = cv::mean(reproj_error);
    return me[0];
}
    
    

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
