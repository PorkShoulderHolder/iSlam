//
//  ViewController.h
//  ios7tester
//
//  Created by Sam Fox on 10/4/13.
//  Copyright (c) 2013 Sam Fox. All rights reserved.
//


#import <Availability.h>


#import <opencv2/opencv.hpp>
using namespace cv;
#import <opencv2/highgui/cap_ios.h>

#import <UIKit/UIKit.h>
#import <Foundation/Foundation.h>
#import "CVDelegate.h"
#import "circleDetector.h"
#import "templateMatcher.h"
#import "OpticalFlow.h"
#import "Calibrator.h"
#import "StructureFromMotion.h"
#import "freak.h"

@interface ViewController : UIViewController <UITableViewDataSource,UITableViewDelegate>

@property (nonatomic,retain) IBOutlet UIImageView *imgView;
@property (nonatomic,retain) CVDelegate *del;
@property (nonatomic,retain) CvVideoCamera *camera;
@property (nonatomic,retain) circleDetector *circleD;
@property (nonatomic,retain) templateMatcher *templateDel;
@property (nonatomic,retain) OpticalFlow *opticalFlow;
@property (nonatomic,retain) Calibrator *calibrate;
@property (nonatomic,retain) freak *freek;
@property (nonatomic,retain) StructureFromMotion *sfm;
@property (nonatomic,retain) IBOutlet UITableView *tableView;

-(IBAction) saveImage:(id)sender;
-(IBAction) run:(id)sender;
-(IBAction) swipeUp:(id)sender;
-(IBAction) swipeDown:(id)sender;


@end
