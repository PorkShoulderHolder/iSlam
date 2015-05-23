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
cv::vector<cv::KeyPoint> keypoints_object1;
float nndrRatio1 = 0.7f;
cv::Matx34d P01;
cv::Mat accT1(0.0f,0.0f,0.0f);
cv::vector<cv::Point3d> pcl1;
cv::Mat cameraIntrinsic1;
cv::vector<cv::DMatch>matches;
cv::Mat distortionMatrix1;
double maxErr = 100000000;
int thresh1 = 34;
int alternator1 = 0;

struct Anchor1 {
    cv::Point3d pt;
    cv::vector<int>index_of_2d_origin;
    cv::Point3d confidence;
};

static void drawArrows1(cv::Mat& frame, const cv::vector<cv::Point2f>& prevPts, const cv::vector<cv::Point2f>& nextPts, const cv::vector<uchar>& status,
                       cv::Scalar line_color = cv::Scalar(0, 0, 255))
{
    int statcnt = 0;
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
               // continue;
            
            // Here we lengthen the arrow by a factor of three.
           q.x = (int) (p.x - 1.0 * hypotenuse * cos(angle));
           q.y = (int) (p.y - 1.0 * hypotenuse * sin(angle));
            
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
            statcnt++;
        }
    }
}

cv::vector<cv::Matx34d> potentialCamMatrices(cv::SVD svd){
    
    // calculates the 4 possible solutions for the camera matrix for vetting
    
    cv::vector<cv::Matx34d> candidates;
    cv::Matx34d c1, c2, c3, c4;
    cv::Matx33d W(0,-1,0,//HZ 9.13
                  1,0,0,
                  0,0,1);
    
    cv::Mat R1 = svd.u * cv::Mat(W) * svd.vt;
    cv::Mat T1 = svd.u.col(2);
    cv::hconcat(R1,T1,c1);
    
    cv::Mat R2 = R1.clone();
    cv::Mat T2 = -1 * svd.u.col(2);
    cv::hconcat(R2,T2,c2);

    
    cv::Mat R3 = svd.u * cv::Mat(W).t() * svd.vt;
    cv::Mat T3 = T1.clone();
    cv::hconcat(R3,T3,c3);

    cv::Mat R4 = R3.clone();
    cv::Mat T4 = T2.clone();
    cv::hconcat(R4,T4,c4);
    
    candidates.push_back(c1);
    candidates.push_back(c2);
    candidates.push_back(c3);
    candidates.push_back(c4);
    
    return candidates;
}

int testCamMatrix(cv::Matx34d camMat1, cv::Matx34d camMat2, cv::vector<cv::Point3d> pointcloud){
    
    // do the points lie in front of the cameras? how many?
    int pointsInFront = 0;
    for (int i = 0; i < pointcloud.size(); i++) {
        cv::Point3d point = pointcloud[i];
        cv::Mat camMatm1 = cv::Mat(camMat1).t();
        cv::Mat camMatm2 = cv::Mat(camMat2).t();
        cv::Mat pointm = cv::Mat(point);
        
        cv::Mat posWRTCam1 = camMatm1 * pointm;
        cv::Mat posWRTCam2 = camMatm2 * pointm;
        bool inFront = posWRTCam1.at<double>(2) > 0 && posWRTCam2.at<double>(2) > 0;
        pointsInFront += inFront;
    }
    
    return pointsInFront;
}
void KeyPointsToPoints1(const cv::vector<cv::KeyPoint>& kps, cv::vector<cv::Point2f>& ps) {
	ps.clear();
	for (unsigned int i=0; i<kps.size(); i++) ps.push_back(kps[i].pt);
}
void PointsToKeyPoints(const cv::vector<cv::Point2f>& ps, cv::vector<cv::KeyPoint>& kps) {
    kps.clear();
    for (unsigned int i=0; i<ps.size(); i++) kps.push_back(cv::KeyPoint(ps[i],1.0f));
}

bool CheckCoherentRotation1(cv::Mat camMat) {
    cv::Mat R = camMat(cv::Rect(0,0,3,3));
    float det = fabs(cv::determinant(R));
    if(det - 1.0 > 1e-07) {
        std::cerr<<"det(R) != +-1.0, this is not a rotation matrix"<<std::endl;
        return false;
    }
    return true;
}
struct CloudPoint1 {
    cv::Point3d pt;
    cv::vector<int>index_of_2d_origin;
};


cv::Mat_<double> LinearLSTriangulation1(
                                       cv::Point3d u,//homogenous image point (u,v,1)
                                       cv::Matx34d P,//camera 1 matrix
                                       cv::Point3d u1,//homogenous image point in 2nd camera
                                       cv::Matx34d P1//camera 2 matrix
)
{
    //build A
    cv::Matx43d A(u.x*P(2,0)-P(0,0),u.x*P(2,1)-P(0,1),u.x*P(2,2)-P(0,2),
                  u.y*P(2,0)-P(1,0),u.y*P(2,1)-P(1,1),u.y*P(2,2)-P(1,2),
                  u1.x*P1(2,0)-P1(0,0), u1.x*P1(2,1)-P1(0,1),u1.x*P1(2,2)-P1(0,2),
                  u1.y*P1(2,0)-P1(1,0), u1.y*P1(2,1)-P1(1,1),u1.y*P1(2,2)-P1(1,2));
    //build B
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
                         const cv::vector<cv::Point2f>& pt_set1,
                         const cv::vector<cv::Point2f>& pt_set2,
                         const cv::Matx34d& P,
                         const cv::Matx34d& P1,
                         cv::vector<cv::Point3d>& pointcloud)
{
    cv::vector<double> reproj_error;
    for (unsigned int i=0; i<fmin(pt_set1.size(), pt_set2.size()); i++) {
        cv::Point2f kp = pt_set1[i];
        cv::Point3d u(kp.x,kp.y,1.0);
        cv::Mat_<double> um = cameraIntrinsic1.inv() * cv::Mat_<double>(u);
        u = um.at<cv::Point3d>(0);
        cv::Point2f kp1 = pt_set2[i];
        cv::Point3d u1(kp1.x,kp1.y,1.0);
        cv::Mat_<double> um1 = cameraIntrinsic1.inv() * cv::Mat_<double>(u1);
        u1 = um1.at<cv::Point3d>(0);
        
        //triangulate
        cv::Mat_<double> X = LinearLSTriangulation1(u,P,u1,P1);
        //
        // calculate reprojection error
        cv::Mat p1mat = cv::Mat(P1);
        cv::Mat m1 = cameraIntrinsic1 * cv::Mat(P1);
        cv::Mat_<double> xPt_img = m1.t() * X;
        cv::Point2f xPt_img_(xPt_img(0)/xPt_img(2),xPt_img(1)/xPt_img(2));
        reproj_error.push_back(norm(xPt_img_-kp1));
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
        
        self.openGLView = [[OpenGLView alloc] initWithFrame:CGRectMake(0, 0, 350, 350)];
        self.openGLView.userInteractionEnabled = YES;
        self.shouldSave = YES;
        self.shouldRun = YES;
    }
    return self;
}

-(void)processImage:(cv::Mat &)image{
    if(self.shouldRun){
    cv::Mat img_scene;
    cv::Mat cpy;
    cv::cvtColor(image, cpy, CV_BGR2GRAY);
    
    cv::undistort(cpy, img_scene, cameraIntrinsic1, distortionMatrix1);
    
    if(self.shouldSave){
        maxErr = 10000000;
        img_object1 = img_scene;
        self.shouldSave = NO;
    }
        
    cv::vector<cv::KeyPoint>left_keypoints;
    cv::vector<cv::Point2f>lp;

    // Detect keypoints in the left and right images
       
    //cv::FAST(img_object1, left_keypoints, 28);
    cv::goodFeaturesToTrack(img_object1, lp, 1100, 0.001, 4);
        
    PointsToKeyPoints(lp, left_keypoints);

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
        
    cv::vector<cv::Point2f>imgpts1,imgpts2;
    cv::vector<cv::KeyPoint>kpts1,kpts2;
    
    for( unsigned int i = 0; i<left_points.size(); i++ )
    {
        if(vstatus[i]){
            imgpts1.push_back(left_points[i]);
            imgpts2.push_back(right_points[i]);
        }
    }
    
        PointsToKeyPoints(imgpts1, kpts1);
        PointsToKeyPoints(imgpts2, kpts2);
    
    cv::vector<uchar> status;
    if (imgpts1.size() > 0) {
        cv::Mat F = findFundamentalMat(imgpts1, imgpts2, cv::FM_RANSAC, 3, 0.9994,status);
        cv::Mat_<double> E = cameraIntrinsic1.t() * F * cameraIntrinsic1; //according to HZ (9.12)
        
        cv::SVD svd(E);
        
        cv::vector<cv::Point3d> verifiedPoints;
        int maxGoodPoints = 0;
        cv::vector<cv::Matx34d> candidates = potentialCamMatrices(svd);
        for (int i = 0; i < 4; i++) {
            sleep(0.1);
            
            cv::vector<cv::Point3d> pointcloud;
            double err = TriangulatePoints1(imgpts1, imgpts2, P01, candidates[i], pointcloud);
            int infrontCount = testCamMatrix(P01, candidates[i], pointcloud);
            if(maxGoodPoints < infrontCount && CheckCoherentRotation1(cv::Mat(candidates[i]) )){
                verifiedPoints = pointcloud;
                maxErr = err;

                maxGoodPoints = infrontCount;
            }
            
        }
        cv::Mat pointsMat = cv::Mat::zeros(verifiedPoints.size(), 3, CV_32F);
        cv::vector<cv::Scalar> colors;
        int avgingDim = 5;
        cv::Mat paddedCopy;
        cv::copyMakeBorder(image, paddedCopy, avgingDim/2, avgingDim/2, avgingDim/2, avgingDim/2, cv::BORDER_REFLECT);
        for (int i = 0; i < verifiedPoints.size(); i++) {
            cv::Point2f kp = kpts2[i].pt;
            
            // no need to offset x,y because of padding
            if (kp.x < paddedCopy.cols - avgingDim && kp.y < paddedCopy.rows - avgingDim && kp.x > avgingDim && kp.y > avgingDim) {
                cv::Rect roi(kp.x, kp.y, avgingDim, avgingDim);
                
                cv::Mat area = paddedCopy(roi);
                cv::Scalar rgb = cv::mean(area);
                colors.push_back(rgb);
                
            }
            else{
                colors.push_back(cv::Scalar(255,255,255));
            }
            
            pointsMat.at<double>(i,0) = verifiedPoints[i].x;
            pointsMat.at<double>(i,1) = verifiedPoints[i].y;
            pointsMat.at<double>(i,2) = verifiedPoints[i].z;
            
        }
        
        cv::Mat e = cv::Mat::zeros(img_object1.rows, img_object2.cols, img_object1.type());

        
        drawArrows1(image, imgpts1, imgpts2, status);
        
        
        
        if (verifiedPoints.size() > 4) {
            sleep(0.2);
            self.openGLView.colorsToRender = colors;
            self.openGLView.pixelsToRender = verifiedPoints;
        }
        //if (self.shouldSave){
            img_object1 = img_scene;
        //}
    }
}
    
}


@end
