//
//  OpenGLView.h
//  ColorSpacious
//
//  Created by Sam Fox on 6/16/13.
//  Copyright (c) 2013 Sam Fox. All rights reserved.
//

#import <UIKit/UIKit.h>
#import <QuartzCore/QuartzCore.h>
#import <OpenGLES/ES2/gl.h>
#import <OpenGLES/ES2/glext.h>
#import "CC3GLMatrix.h"
#import "OpticalFlow.h"


@interface OpenGLView : UIView

@property (nonatomic,retain)CAEAGLLayer* eaglLayer;
@property (nonatomic,retain)EAGLContext* context;
@property GLuint colorRenderBuffer;
@property GLuint colorSlot;
@property GLuint positionSlot;
@property GLuint projectionUniform;
@property GLuint modelViewUniform;

@property float currentRotation;
@property (nonatomic, assign) float zoom;
@property (atomic, assign) cv::vector<cv::Point3d> pixelsToRender;
@property (atomic, assign) cv::vector<cv::Point3d> colorsToRender;


@property (nonatomic, retain) UIGestureRecognizer *gestureRecognizer;
@property (nonatomic, assign) CGPoint touchDown;
@property (nonatomic, readwrite) BOOL dontTouch;

- (std::vector<float>)getPixelsWithHorizDensity:(int)xres andVertDensity:(int)yres;

+(Class)layerClass;

@end
