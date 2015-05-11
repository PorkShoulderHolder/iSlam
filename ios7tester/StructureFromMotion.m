//
//  StructureFromMotion.m
//  ios7tester
//
//  Created by Sam Royston on 5/10/15.
//  Copyright (c) 2015 Sam Fox. All rights reserved.
//

#import "StructureFromMotion.h"

@implementation StructureFromMotion

cv::Mat img_object1,homography1, img_object2;
cv::Mat descriptors_object1;
std::vector<cv::KeyPoint> keypoints_object1;
float nndrRatio1 = 0.7f;
cv::Matx34d P01;
cv::Mat accT1(0.0f,0.0f,0.0f);
cv::vector<cv::Point3d> pcl1;
cv::Mat cameraIntrinsic1;
cv::vector<cv::DMatch>matches;
cv::Mat distortionMatrix1;

int thresh1 = 14;
int alternator1 = 0;

struct Anchor1 {
    cv::Point3d pt;
    std::vector<int>index_of_2d_origin;
    cv::Point3d confidence;
};

static void drawArrows1(cv::Mat frame, const cv::vector<cv::Point2f>& prevPts, const cv::vector<cv::Point2f>& nextPts, const cv::vector<uchar>& status,
                       cv::Scalar line_color = cv::Scalar(0, 0, 255))
{
    for (size_t i = 0; i < prevPts.size(); ++i)
    {
        if (status[i])
        {
            int line_thickness = 1;
            
            cv::Point p = prevPts[i];
            cv::Point q = nextPts[i];
            
            double angle = atan2((double) p.y - q.y, (double) p.x - q.x);
            
            double hypotenuse = sqrt( (double)(p.y - q.y)*(p.y - q.y) + (double)(p.x - q.x)*(p.x - q.x) );
            
            if (hypotenuse < 1.0)
                continue;
            
            // Here we lengthen the arrow by a factor of three.
            q.x = (int) (p.x - 3 * hypotenuse * cos(angle));
            q.y = (int) (p.y - 3 * hypotenuse * sin(angle));
            
            // Now we draw the main line of the arrow.
            line(frame, p, q, line_color, line_thickness);
            
            // Now draw the tips of the arrow. I do some scaling so that the
            // tips look proportional to the main line of the arrow.
            
            p.x = (int) (q.x + 9 * cos(angle + CV_PI / 4));
            p.y = (int) (q.y + 9 * sin(angle + CV_PI / 4));
            line(frame, p, q, line_color, line_thickness);
            
            p.x = (int) (q.x + 9 * cos(angle - CV_PI / 4));
            p.y = (int) (q.y + 9 * sin(angle - CV_PI / 4));
            line(frame, p, q, line_color, line_thickness);
        }
    }
}


void KeyPointsToPoints1(const cv::vector<cv::KeyPoint>& kps, cv::vector<cv::Point2f>& ps) {
	ps.clear();
	for (unsigned int i=0; i<kps.size(); i++) ps.push_back(kps[i].pt);
}

bool CheckCoherentRotation1(cv::Mat_<double>& R) {
    if(fabs(determinant(R))-1.0 > 1e-07) {
        std::cerr<<"det(R) != +-1.0, this is not a rotation matrix"<<std::endl;
        return false;
    }
    return true;
}
struct CloudPoint1 {
    cv::Point3d pt;
    std::vector<int>index_of_2d_origin;
};

void FindCameraMatrices(const cv::Mat& K,
                        const cv::Mat& Kinv,
                        const cv::vector<cv::KeyPoint>& imgpts1,
                        const cv::vector<cv::KeyPoint>& imgpts2,
                        cv::Matx34d& P,
                        cv::Matx34d& P1,
                        cv::vector<cv::DMatch>& matches,
                        cv::vector<CloudPoint1>& outCloud
                        )
{
    //Find camera matrices
    //Get Fundamental Matrix
    cv::Mat F = cv::findFundamentalMat(imgpts1, imgpts2, matches);
    //Essential matrix: compute then extract cameras [R|t]
    cv::Mat_<double> E = K.t() * F * K; //according to HZ (9.12)
    //decompose E to P' , HZ (9.19)
    cv::SVD svd(E,cv::SVD::MODIFY_A);
    cv::Mat svd_u = svd.u;
    cv::Mat svd_vt = svd.vt;
    cv::Mat svd_w = svd.w;
    cv::Matx33d W(0,-1,0,//HZ 9.13
              1,0,0,
              0,0,1);
    
    
    cv::Mat_<double> R = svd_u * cv::Mat(W) * svd_vt; //HZ 9.19
    cv::Mat_<double> t = svd_u.col(2); //u3
    if (!CheckCoherentRotation1(R)) {
        std::cout<<"resulting rotation is not coherent\n";
        P1 = 0;
        return;
    }
    P1 = cv::Matx34d(R(0,0),R(0,1),R(0,2),t(0),
                 R(1,0),R(1,1),R(1,2),t(1),
                 R(2,0),R(2,1),R(2,2),t(2));
}

cv::Mat_<double> LinearLSTriangulation1(
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

double TriangulatePoints1(
                         const cv::vector<cv::KeyPoint>& pt_set1,
                         const cv::vector<cv::KeyPoint>& pt_set2,
                         // const cv::Mat&Kinv,
                         const cv::Matx34d& P,
                         const cv::Matx34d& P1,
                         cv::vector<cv::Point3d>& pointcloud)
{
    cv::vector<double> reproj_error;
    for (unsigned int i=0; i<fmin(pt_set1.size(), pt_set2.size()); i++) {
        cv::Point2f kp = pt_set1[i].pt;
        cv::Point3d u(kp.x,kp.y,1.0);
        cv::Mat_<double> um = cameraIntrinsic1.t() * cv::Mat_<double>(u);
        u = um.at<cv::Point3d>(0);
        cv::Point2f kp1 = pt_set2[i].pt;
        cv::Point3d u1(kp1.x,kp1.y,1.0);
        cv::Mat_<double> um1 = cameraIntrinsic1.t() * cv::Mat_<double>(u1);
        u1 = um1.at<cv::Point3d>(0);
        
        //triangulate
        cv::Mat_<double> X = LinearLSTriangulation1(u,P,u1,P1);
        
        //calculate reprojection error
//        cv::Mat_<double> xPt_img = cameraIntrinsic1 * cv::Mat(P1) * X;
//        cv::Point2f xPt_img_(xPt_img(0)/xPt_img(2),xPt_img(1)/xPt_img(2));
//        reproj_error.push_back(norm(xPt_img_-kp1));
        //store 3D point
        pointcloud.push_back(cv::Point3d(X(0),X(1),X(2)));    }
    //return mean reprojection error
    cv::Scalar me = cv::mean(reproj_error);
    return me[0];
    
}

- (id)init{
    if (self == [super init]) {
        
        
#pragma mark TODO: get Mat from uiimage
        
            UIImage *img1 = [UIImage imageNamed:@"0001.png"];
            UIImage *img2 = [UIImage imageNamed:@"0002.png"];
        
        
        img_object1 = [freak cvMatFromUIImage:img1];
        img_object2 = [freak cvMatFromUIImage:img2];
        cv::cvtColor(img_object1, img_object1, CV_BGR2GRAY);
        cv::cvtColor(img_object2, img_object2, CV_BGR2GRAY);

        
        cv::FAST(img_object1, keypoints_object1, 9);

        P01 = cv::Matx34d(1.0,    0.0, 0.0, 0.0,
                         0.0,    1.0, 0.0, 0.0,
                         0.0,    0.0, 1.0, 0.0);
        cameraIntrinsic1 = [Calibrator loadMatFromKey:@"cameraMatrix"];
        distortionMatrix1 = [Calibrator loadMatFromKey:@"distortionMatrix"];
        self.openGLView = [[OpenGLView alloc] initWithFrame:CGRectMake(100, 0, 150, 150)];
        self.openGLView.userInteractionEnabled = YES;
        self.shouldSave = YES;
        self.shouldRun = YES;
    }
    return self;
}

-(void)processImage:(cv::Mat &)image{
    if(self.shouldRun){
    cv::Mat img_scene;
        
    cv::cvtColor(image, img_scene, CV_BGR2GRAY);
        if(self.shouldSave){
            img_object1 = img_scene;
            self.shouldSave = NO;
        }
        
    cv::vector<cv::KeyPoint>left_keypoints,right_keypoints;
    // Detect keypoints in the left and right images
    
       
    cv::FAST(img_object1, left_keypoints, thresh1);
    cv::FAST(img_scene, right_keypoints, thresh1);

  
    if( matches.size() > 400 || right_keypoints.size() > 800){
        thresh1++;
    }
    else if ( matches.size() < 400 && right_keypoints.size() < 800){
        thresh1--;
    }


    cv::vector<cv::Point2f>left_points;
    KeyPointsToPoints1(left_keypoints,left_points);
    cv::vector<cv::Point2f>right_points(left_points.size());
    // making sure images are grayscale
    cv::Mat prevgray,gray;
    prevgray = img_object1;
    gray = img_scene;
    
    
    // Calculate the optical flow field:
    // how each left_point moved across the 2 images
    cv::vector<uchar>vstatus; cv::vector<float>verror;
    calcOpticalFlowPyrLK(prevgray, gray, left_points, right_points,
                         vstatus, verror);
    // First, filter out the points with high error
    cv::vector<cv::Point2f>right_points_to_find;
    cv::vector<int>right_points_to_find_back_index;
    for (unsigned int i=0; i<vstatus.size(); i++) {
        if (vstatus[i] &&verror[i] < 12.0) {
            // Keep the original index of the point in the
            // optical flow array, for future use
            right_points_to_find_back_index.push_back(i);
            // Keep the feature point itself
            right_points_to_find.push_back(right_points[i]);
        } else {
            vstatus[i] = 0; // a bad flow
        }
    }
    // for each right_point see which detected feature it belongs to
    cv::Mat right_points_to_find_flat = cv::Mat(right_points_to_find).reshape(1,right_points_to_find.size()); //flatten array
    cv::vector<cv::Point2f>right_features; // detected features
    KeyPointsToPoints1(right_keypoints,right_features);
    cv::Mat right_features_flat = cv::Mat(right_features).reshape(1,right_features.size());
    // Look around each OF point in the right image
    // for any features that were detected in its area
    // and make a match.
    cv::BFMatcher matcher(CV_L2);
    cv::vector<cv::vector<cv::DMatch>>nearest_neighbors;
    matcher.radiusMatch(right_points_to_find_flat,right_features_flat,nearest_neighbors,2.0f);
    // Check that the found neighbors are unique (throw away neighbors
    // that are too close together, as they may be confusing)
    std::set<int>found_in_right_points; // for duplicate prevention
    
    cv::vector<cv::DMatch> mnew;
    matches = mnew;
    for(int i=0;i<nearest_neighbors.size();i++) {
        cv::DMatch _m;
        if(nearest_neighbors[i].size()==1) {
            _m = nearest_neighbors[i][0]; // only one neighbor
        } else if(nearest_neighbors[i].size()>1) {
            // 2 neighbors – check how close they are
            double ratio = nearest_neighbors[i][0].distance /
            nearest_neighbors[i][1].distance;
            if(ratio < 0.7) { // not too close
                // take the closest (first) one
                _m = nearest_neighbors[i][0];
            } else { // too close – we cannot tell which is better
                continue; // did not pass ratio test – throw away
            }
        } else {
            continue; // no neighbors... :(
        }
        // prevent duplicates
        if (found_in_right_points.find(_m.trainIdx) == found_in_right_points.
            end()) {
            // The found neighbor was not yet used:
            // We should match it with the original indexing
            // ofthe left point
            _m.queryIdx = right_points_to_find_back_index[_m.queryIdx];
            matches.push_back(_m); // add this match
            found_in_right_points.insert(_m.trainIdx);
        }
    }
    cv::vector<cv::Point2f>imgpts1,imgpts2;
    cv::vector<cv::KeyPoint>kpts1,kpts2;
    
    for( unsigned int i = 0; i<matches.size(); i++ )
    {
        // queryIdx is the "left" image
        imgpts1.push_back(left_keypoints[matches[i].queryIdx].pt);
        kpts1.push_back(left_keypoints[matches[i].queryIdx]);

        
        // trainIdx is the "right" image
        imgpts2.push_back(right_keypoints[matches[i].trainIdx].pt);
        kpts2.push_back(right_keypoints[matches[i].queryIdx]);

    }
    
    cv::vector<uchar> status;
    if (imgpts1.size() > 0) {
        cv::Mat F = findFundamentalMat(imgpts1, imgpts2, cv::FM_RANSAC, 0.1, 0.99,status);
        cv::Mat_<double> E = cameraIntrinsic1.t() * F * cameraIntrinsic1; //according to HZ (9.12)
        
        cv::SVD svd(E);
        cv::Matx33d W(0,-1,0,
                      1,0,0,
                      0,0,1);
        
        cv::Mat_<double> R = svd.u * cv::Mat(W) * svd.vt; //HZ 9.19
        cv::Mat_<double> t = svd.u.col(2); //u3
        cv::Matx34d P1( R(0,0),R(0,1), R(0,2), t(0),
                       R(1,0),R(1,1), R(1,2), t(1),
                       R(2,0),R(2,1), R(2,2), t(2));
        cv::vector<cv::Point3d> pointcloud;
        TriangulatePoints1(kpts1, kpts2, P01, P1, pointcloud);
        drawArrows1(image, imgpts1, imgpts2, status);
        self.openGLView.pixelsToRender = pointcloud;
        
            }

    }
    
}


@end
